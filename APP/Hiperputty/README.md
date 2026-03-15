# Toranam Deploy Tool

Mali GUI alat za deploy na server:

- sprema podatke za spajanje
- prikazuje **lokalne** i **remote** fileove
- uspoređuje stanje
- može uploadati samo **odabrane** fileove
- može uploadati samo **promijenjene / nove** fileove
- po želji odradi:
  1. `systemctl stop webapp`
  2. upload
  3. `systemctl start webapp`

## Pokretanje

1. Instaliraj Python 3.10+  
2. U folderu alata pokreni:

```bash
pip install -r requirements.txt
python toranam_deploy_gui.py
```

## Prvo korištenje

Popuni:
- Host / IP
- Port
- Username
- Password
- Sudo password (ako nije root korisnik)
- Local folder
- Remote folder
- Systemd service
- Test URL

Klikni **Save settings**.

## Kako radi upload

### Upload selected
Odaberi jedan ili više fileova u lijevom panelu pa klikni **Upload selected**.

### Upload changed
Alat uspoređuje:
- relativnu putanju
- veličinu filea
- vrijeme izmjene

Ako remote file ne postoji ili se razlikuje, ide u upload.

## Gdje sprema postavke

Postavke se spremaju u:

- Windows: `C:\Users\TvojeIme\.toranam_deploy_gui.json`
- Linux/macOS: `~/.toranam_deploy_gui.json`

Ako je uključeno **Remember passwords locally**, lozinke se spremaju u taj JSON file.

## Preporuka

Za sigurniju varijantu kasnije:
- prijeđi na SSH key
- koristi poseban deploy user
- izbjegni root login ako možeš
