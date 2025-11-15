#!/usr/bin/env bash
set -euo pipefail

echo "======================================="
echo "  Ubuntu 22.04 System UN-Freeze Script"
echo "  - Restore APT repos (if backup exists)"
echo "  - Re-enable auto updates (optional)"
echo "  - Unhold packages (allow upgrades)"
echo "======================================="
echo

if [[ $(id -u) -ne 0 ]]; then
  echo "Harus dijalankan sebagai root. Gunakan:"
  echo "  sudo $0"
  exit 1
fi

echo "Script ini AKAN membuka blokir update sistem."
echo "Artinya: apt update/upgrade akan bisa jalan lagi."
read -p "Ketik 'UNFREEZE' (huruf besar) untuk lanjut: " ANSW
if [[ "$ANSW" != "UNFREEZE" ]]; then
  echo "Dibatalkan."
  exit 0
fi

########################################
# 1. Cari backup freeze sebelumnya
########################################
echo
echo "==> Mencari backup freeze sebelumnya di /root/system-freeze-backup-* ..."

BACKUP_CANDIDATES=(/root/system-freeze-backup-*)
BACKUP_DIR=""

if ls /root/system-freeze-backup-* >/dev/null 2>&1; then
  # Ambil yang terbaru (berdasarkan nama, biasanya sudah urut waktu)
  BACKUP_DIR="$(ls -d /root/system-freeze-backup-* 2>/dev/null | sort | tail -n 1)"
  echo "  Backup terbaru terdeteksi: $BACKUP_DIR"
else
  echo "  Tidak ditemukan direktori /root/system-freeze-backup-*"
  echo "  Unfreeze tetap bisa jalan, tapi restore sources.list harus manual kalau mau."
fi

########################################
# 2. Restore konfigurasi APT (auto-updates)
########################################
echo
echo "==> Restore konfigurasi auto-update APT (jika backup tersedia)..."

restore_file_if_exists() {
  local src="$1"
  local dst="$2"
  if [[ -f "$src" ]]; then
    echo "  Restore: $src -> $dst"
    cp -a "$src" "$dst"
  else
    echo "  [SKIP] Backup tidak ditemukan: $src"
  fi
}

if [[ -n "$BACKUP_DIR" ]]; then
  restore_file_if_exists "$BACKUP_DIR/20auto-upgrades" /etc/apt/apt.conf.d/20auto-upgrades
  restore_file_if_exists "$BACKUP_DIR/10periodic"      /etc/apt/apt.conf.d/10periodic
else
  echo "  Tidak ada backup freeze yang jelas. Melewati restore 20auto-upgrades/10periodic."
fi

########################################
# 3. Re-enable systemd timers & services
########################################
echo
echo "==> Mengaktifkan kembali layanan auto-update (apt-daily, unattended-upgrades)..."

for svc in \
  apt-daily.service apt-daily.timer \
  apt-daily-upgrade.service apt-daily-upgrade.timer \
  unattended-upgrades.service \
  ; do
  if systemctl list-unit-files | grep -q "^$svc"; then
    systemctl enable "$svc" 2>/dev/null || true
    systemctl start "$svc" 2>/dev/null || true
    echo "  Enabled & started: $svc"
  fi
done

########################################
# 4. Restore APT repositories
########################################
echo
echo "==> Restore APT repositories (sources.list & sources.list.d) jika backup tersedia..."

if [[ -n "$BACKUP_DIR" ]]; then
  # sources.list
  if [[ -f "$BACKUP_DIR/sources.list" ]]; then
    echo "  Restore /etc/apt/sources.list dari backup..."
    cp -a "$BACKUP_DIR/sources.list" /etc/apt/sources.list
  else
    echo "  [INFO] Backup sources.list tidak ditemukan di $BACKUP_DIR."
    echo "         Kalau sebelumnya kamu backup ke nama lain, restore manual mungkin diperlukan."
  fi

  # sources.list.d
  if [[ -d "$BACKUP_DIR/sources.list.d" ]]; then
    echo "  Restore /etc/apt/sources.list.d dari backup..."
    rm -rf /etc/apt/sources.list.d
    cp -a "$BACKUP_DIR/sources.list.d" /etc/apt/sources.list.d
  else
    echo "  [INFO] Backup sources.list.d tidak ditemukan di $BACKUP_DIR."
    echo "         Bisa tetap jalan tanpa ini, tapi repo tambahan mungkin hilang."
  fi
else
  echo "  Tidak ada BACKUP_DIR. Jika /etc/apt/sources.list sekarang kosong,"
  echo "  kamu perlu mengisi repo ulang (misalnya pakai default Ubuntu 22.04)."
fi

########################################
# 5. Unhold packages
########################################
echo
echo "==> Menghapus status HOLD dari paket-paket..."

ALL_PKGS_FILE=""

if [[ -n "$BACKUP_DIR" ]] && [[ -f "$BACKUP_DIR/all-packages-list.txt" ]]; then
  ALL_PKGS_FILE="$BACKUP_DIR/all-packages-list.txt"
  echo "  Menggunakan daftar paket dari backup: $ALL_PKGS_FILE"
fi

if [[ -n "$ALL_PKGS_FILE" ]]; then
  echo "  Meng-UNHOLD semua paket di daftar backup (via xargs)..."
  xargs -r apt-mark unhold < "$ALL_PKGS_FILE"
else
  echo "  Tidak ada all-packages-list.txt dari backup."
  echo "  Fallback: UNHOLD semua paket yang saat ini statusnya HOLD..."
  apt-mark showhold | xargs -r apt-mark unhold
fi

echo
echo "==> Contoh paket yang sekarang TIDAK HOLD lagi:"
apt-mark showhold | head || echo "  (mungkin sudah tidak ada paket HOLD)"

echo
echo "======================================="
echo "  Selesai UNFREEZE."
echo "  Sekarang apt update/upgrade bisa jalan lagi."
echo "  Review ulang sources.list kalau perlu (misalnya ganti mirror ke yang cepat)."
echo "======================================="
