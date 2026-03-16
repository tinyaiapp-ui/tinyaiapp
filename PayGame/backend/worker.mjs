import 'dotenv/config';
import fs from 'node:fs';
import path from 'node:path';
import bs58 from 'bs58';
import {
  Connection,
  Keypair,
  PublicKey,
  sendAndConfirmTransaction,
  Transaction
} from '@solana/web3.js';
import {
  getMint,
  getOrCreateAssociatedTokenAccount,
  createTransferCheckedInstruction
} from '@solana/spl-token';

const RPC_URL = process.env.RPC_URL;
const RECIPIENT_WALLET = new PublicKey(process.env.RECIPIENT_WALLET);
const TOKEN_MINT = new PublicKey(process.env.TOKEN_MINT);
const PAYOUT_PRIVATE_KEY_B58 = process.env.PAYOUT_PRIVATE_KEY_B58;
const ROUND_STEP_MINUTES = Number(process.env.ROUND_STEP_MINUTES || 5);
const PAYOUT_PERCENT = Number(process.env.PAYOUT_PERCENT || 95);
const POLL_SECONDS = Number(process.env.POLL_SECONDS || 20);

if (!RPC_URL || !PAYOUT_PRIVATE_KEY_B58) {
  throw new Error('Missing RPC_URL or PAYOUT_PRIVATE_KEY_B58 in .env');
}

const payer = Keypair.fromSecretKey(bs58.decode(PAYOUT_PRIVATE_KEY_B58));
const connection = new Connection(RPC_URL, 'confirmed');
const statePath = path.join(process.cwd(), 'state.json');

function readState() {
  try {
    return JSON.parse(fs.readFileSync(statePath, 'utf8'));
  } catch {
    return { paidRounds: {} };
  }
}

function writeState(state) {
  fs.writeFileSync(statePath, JSON.stringify(state, null, 2));
}

function floorToStep(date, stepMin) {
  const d = new Date(date);
  d.setSeconds(0, 0);
  const m = d.getMinutes();
  d.setMinutes(Math.floor(m / stepMin) * stepMin);
  return d;
}

function nextRoundCloseMs(stepMin) {
  const now = new Date();
  const floored = floorToStep(now, stepMin);
  floored.setMinutes(floored.getMinutes() + stepMin);
  return floored.getTime();
}

function roundStamp(closeMs) {
  const d = new Date(closeMs);
  const y = d.getFullYear();
  const mo = String(d.getMonth() + 1).padStart(2, '0');
  const da = String(d.getDate()).padStart(2, '0');
  const h = String(d.getHours()).padStart(2, '0');
  const mi = String(d.getMinutes()).padStart(2, '0');
  return `${y}${mo}${da}${h}${mi}`;
}

function memoForClose(closeMs) {
  return `PAYGAME|ROUND_${roundStamp(closeMs)}|ENTRY`;
}

function extractMemo(tx) {
  const ixs = tx?.transaction?.message?.instructions || [];
  for (const ix of ixs) {
    if (ix?.program === 'spl-memo') {
      if (typeof ix.parsed === 'string') return ix.parsed;
      if (typeof ix.parsed?.memo === 'string') return ix.parsed.memo;
    }
  }
  return '';
}

function extractSender(tx, fallbackRecipient) {
  const keys = tx?.transaction?.message?.accountKeys || [];
  const signer = keys.find(k => k.signer && k.pubkey !== fallbackRecipient.toBase58());
  return signer?.pubkey || keys?.[0]?.pubkey || null;
}

function tokenDeltaToRecipient(tx, recipient, mint) {
  const pre = tx?.meta?.preTokenBalances || [];
  const post = tx?.meta?.postTokenBalances || [];
  const key = (x) => `${x.accountIndex}|${x.owner}|${x.mint}`;
  const map = new Map();

  pre.forEach(p => map.set(key(p), -Number(p.uiTokenAmount?.uiAmount || 0)));
  post.forEach(p => map.set(key(p), (map.get(key(p)) || 0) + Number(p.uiTokenAmount?.uiAmount || 0)));

  let delta = 0;
  const needle = `|${recipient.toBase58()}|${mint.toBase58()}`;
  for (const [k, v] of map.entries()) {
    if (k.includes(needle)) delta += v;
  }
  return delta;
}

async function getRoundEntries(closeMs) {
  const startMs = closeMs - ROUND_STEP_MINUTES * 60 * 1000;
  const targetMemo = memoForClose(closeMs);
  const sigs = await connection.getSignaturesForAddress(RECIPIENT_WALLET, { limit: 150 });
  const inWindow = sigs.filter(s => {
    const t = (s.blockTime || 0) * 1000;
    return t >= startMs - 15000 && t <= closeMs + 15000;
  });

  const txs = await Promise.all(
    inWindow.map(s => connection.getTransaction(s.signature, { maxSupportedTransactionVersion: 0 }))
  );

  const entries = txs.map(tx => {
    if (!tx) return null;
    const memo = extractMemo(tx);
    if (memo !== targetMemo) return null;
    const amount = tokenDeltaToRecipient(tx, RECIPIENT_WALLET, TOKEN_MINT);
    if (amount <= 0) return null;
    const sender = extractSender(tx, RECIPIENT_WALLET);
    if (!sender) return null;
    return {
      sender,
      amount,
      timeMs: (tx.blockTime || 0) * 1000,
      signature: tx.transaction.signatures[0]
    };
  }).filter(Boolean);

  entries.sort((a, b) => b.timeMs - a.timeMs); // latest first
  return entries;
}

async function payoutWinner(winnerAddress, payoutUiAmount) {
  const mintInfo = await getMint(connection, TOKEN_MINT);
  const decimals = mintInfo.decimals;

  const fromAta = await getOrCreateAssociatedTokenAccount(
    connection,
    payer,
    TOKEN_MINT,
    payer.publicKey
  );

  const toAta = await getOrCreateAssociatedTokenAccount(
    connection,
    payer,
    TOKEN_MINT,
    new PublicKey(winnerAddress)
  );

  const units = BigInt(Math.floor(payoutUiAmount * 10 ** decimals));
  if (units <= 0n) throw new Error('Payout units <= 0');

  const ix = createTransferCheckedInstruction(
    fromAta.address,
    TOKEN_MINT,
    toAta.address,
    payer.publicKey,
    units,
    decimals
  );

  const tx = new Transaction().add(ix);
  const sig = await sendAndConfirmTransaction(connection, tx, [payer]);
  return sig;
}

async function processLastClosedRound() {
  const state = readState();
  const currentClose = nextRoundCloseMs(ROUND_STEP_MINUTES);
  const lastClosed = currentClose - ROUND_STEP_MINUTES * 60 * 1000;
  const roundKey = roundStamp(lastClosed);

  if (state.paidRounds[roundKey]) return;

  const now = Date.now();
  if (now < lastClosed + 15000) return; // small delay for finality

  const entries = await getRoundEntries(lastClosed);
  if (!entries.length) {
    state.paidRounds[roundKey] = { status: 'no_entries', at: new Date().toISOString() };
    writeState(state);
    console.log(`[${roundKey}] no entries`);
    return;
  }

  const winner = entries[0]; // latest payer
  const pool = entries.reduce((s, e) => s + e.amount, 0);
  const payoutAmount = (pool * PAYOUT_PERCENT) / 100;

  try {
    const payoutSig = await payoutWinner(winner.sender, payoutAmount);
    state.paidRounds[roundKey] = {
      status: 'paid',
      winner: winner.sender,
      entries: entries.length,
      pool,
      payoutAmount,
      payoutPercent: PAYOUT_PERCENT,
      payoutSignature: payoutSig,
      at: new Date().toISOString()
    };
    writeState(state);
    console.log(`[${roundKey}] paid winner=${winner.sender} payout=${payoutAmount} sig=${payoutSig}`);
  } catch (err) {
    state.paidRounds[roundKey] = {
      status: 'error',
      winner: winner.sender,
      entries: entries.length,
      pool,
      payoutAmount,
      error: String(err?.message || err),
      at: new Date().toISOString()
    };
    writeState(state);
    console.error(`[${roundKey}] payout error`, err);
  }
}

console.log('PayGame worker started');
console.log('payer=', payer.publicKey.toBase58());
setInterval(() => {
  processLastClosedRound().catch(e => console.error('loop error', e));
}, POLL_SECONDS * 1000);

processLastClosedRound().catch(e => console.error('init error', e));
