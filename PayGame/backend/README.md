# PayGame Auto Payout Worker

This worker pays the **last valid entry** of each closed 5-minute round.

## Rules implemented
- Round closes every 5 minutes (`:00, :05, :10...`)
- Valid entry = incoming transfer of your token to `RECIPIENT_WALLET` with matching memo format
- Winner = latest valid entry before close
- Payout = `PAYOUT_PERCENT` (set to `95`) of the round pool

## Setup
1. Open this folder:
   - `C:\Users\Korisnik\Documents\GitHub\tinyaiapp\PayGame\backend`
2. Install deps:
   ```bash
   npm install
   ```
3. Create `.env` from `.env.example` and fill secret key:
   - `PAYOUT_PRIVATE_KEY_B58` (base58 private key for payout wallet)
4. Start worker:
   ```bash
   npm start
   ```

## Important
- Keep enough token balance in payout wallet ATA.
- Keep some SOL in payout wallet for fees.
- `state.json` prevents double-payout per round.

## Logs
Worker writes payout/no-entry/error info to `state.json`.
