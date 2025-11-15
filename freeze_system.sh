#!/usr/bin/env bash
set -euo pipefail

echo "======================================="
echo "  Ubuntu 22.04 System Freeze Script"
echo "  - Disable auto updates"
echo "  - Hold all packages (no upgrades)"
echo "  - Freeze kernel/firmware/NVIDIA"
echo "  - Disable APT repositories"
echo "======================================="
echo

if [[ $(id -u) -ne 0 ]]; then
  echo "Harus dijalankan sebagai root. Gunakan:"
  echo "  sudo $0"
  exit 1
fi

read -p "Ini akan MEMBEKUKAN sistem dari update. Ketik 'YES' (huruf besar) untuk lanjut: " ANSW
if [[ "$ANSW" != "YES" ]]; then
  echo "Dibatalkan."
  exit 1
fi

TIMESTAMP=$(date +"%Y%m%d-%H%M%S")
BACKUP_DIR="/root/system-freeze-backup-$TIMESTAMP"
mkdir -p "$BACKUP_DIR"

echo
echo "Backup konfigurasi penting ke: $BACKUP_DIR"
echo

backup_file() {
  local f="$1"
  if [[ -f "$f" ]]; then
    cp -a "$f" "$BACKUP_DIR"/
    echo "  Backup: $f -> $BACKUP_DIR/"
  fi
}

backup_dir() {
  local d="$1"
  if [[ -d "$d" ]]; then
    cp -a "$d" "$BACKUP_DIR"/
    echo "  Backup dir: $d -> $BACKUP_DIR/"
  fi
}

########################################
# 1. Disable automatic APT updates
########################################
echo
echo "==> Menonaktifkan layanan auto-update APT & unattended-upgrades..."

# Disable timers & services
for svc in \
  apt-daily.service apt-daily.timer \
  apt-daily-upgrade.service apt-daily-upgrade.timer \
  unattended-upgrades.service \
  ; do
  if systemctl list-unit-files | grep -q "^$svc"; then
    systemctl disable --now "$svc" 2>/dev/null || true
    echo "  Disabled: $svc"
  fi
done

# Backup configs
backup_file /etc/apt/apt.conf.d/20auto-upgrades
backup_file /etc/apt/apt.conf.d/10periodic

echo
echo "==> Mengatur /etc/apt/apt.conf.d/20auto-upgrades ke mode OFF..."

cat >/etc/apt/apt.conf.d/20auto-upgrades <<EOF
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
EOF

echo "==> Mengatur /etc/apt/apt.conf.d/10periodic ke mode OFF..."

cat >/etc/apt/apt.conf.d/10periodic <<EOF
APT::Periodic::Enable "0";
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Download-Upgradeable-Packages "0";
APT::Periodic::AutocleanInterval "0";
APT::Periodic::Unattended-Upgrade "0";
EOF

########################################
# 2. Hold all packages (freeze upgrades)
########################################
echo
echo "==> Meng-HOLD semua paket yang terinstall (bisa butuh beberapa detik)..."

# Simpan daftar paket
ALL_PKGS_FILE="$BACKUP_DIR/all-packages-list.txt"
dpkg-query -W -f='${binary:Package}\n' | sort -u > "$ALL_PKGS_FILE"
echo "  Daftar semua paket tersimpan di: $ALL_PKGS_FILE"

# Simpan daftar paket
ALL_PKGS_FILE="$BACKUP_DIR/all-packages-list.txt"
dpkg-query -W -f='${binary:Package}\n' | sort -u > "$ALL_PKGS_FILE"
echo "  Daftar semua paket tersimpan di: $ALL_PKGS_FILE"

# Hold semua paket sekaligus (jauh lebih cepat)
xargs -r apt-mark hold < "$ALL_PKGS_FILE"

echo "  Semua paket dicoba untuk di-HOLD (via xargs)."
echo "  Contoh paket yang di-HOLD:"
head -n 10 "$ALL_PKGS_FILE" | sed 's/^/    /'

########################################
# 3. Freeze kernel, headers, firmware, nvidia/cuda
########################################
echo
echo "==> Meng-HOLD kernel & headers saat ini..."

CURRENT_KERNEL="$(uname -r || true)"

if [[ -n "$CURRENT_KERNEL" ]]; then
  apt-mark hold "linux-image-$CURRENT_KERNEL" 2>/dev/null || true
  apt-mark hold "linux-headers-$CURRENT_KERNEL" 2>/dev/null || true
  echo "  Held: linux-image-$CURRENT_KERNEL"
  echo "  Held: linux-headers-$CURRENT_KERNEL"
fi

apt-mark hold linux-image-generic 2>/dev/null || true
apt-mark hold linux-headers-generic 2>/dev/null || true
echo "  Held: linux-image-generic, linux-headers-generic (jika ada)"

echo
echo "==> Meng-HOLD linux-firmware (jika terinstall)..."
apt-mark hold linux-firmware 2>/dev/null || true

echo
echo "==> Meng-HOLD driver NVIDIA dan paket CUDA (jika ada)..."
# Ini pola generic, kalau tidak ada tidak masalah
for pattern in "nvidia-driver-" "cuda-" "libcudnn" "libnvinfer"; do
  pkgs=$(dpkg -l | awk '{print $2}' | grep -E "^${pattern}" || true)
  if [[ -n "$pkgs" ]]; then
    for p in $pkgs; do
      apt-mark hold "$p" 2>/dev/null || true
      echo "  Held: $p"
    done
  fi
done

########################################
# 4. Disable APT repositories
########################################
echo
echo "==> Menonaktifkan semua repository APT (sources.list & sources.list.d)..."

backup_file /etc/apt/sources.list
backup_dir  /etc/apt/sources.list.d

# Ganti sources.list jadi kosong
echo "# frozen: all repos disabled on $TIMESTAMP" >/etc/apt/sources.list

# Rename sources.list.d lama, dan bikin dir kosong baru
if [[ -d /etc/apt/sources.list.d ]]; then
  mv /etc/apt/sources.list.d "/etc/apt/sources.list.d.frozen-$TIMESTAMP"
fi
mkdir -p /etc/apt/sources.list.d

echo "  /etc/apt/sources.list dikosongkan."
echo "  /etc/apt/sources.list.d dipindah ke /etc/apt/sources.list.d.frozen-$TIMESTAMP dan dibuat ulang kosong."

########################################
# 5. Show final status
########################################
echo
echo "==> Status paket yang di-HOLD (contoh):"
apt-mark showhold | head -n 50 | sed 's/^/  /'

echo
echo "======================================="
echo "  Selesai."
echo "  Sistem kamu sekarang DIFREEZE dari update via APT."
echo
echo "  Backup konfigurasi ada di: $BACKUP_DIR"
echo
echo "  Kalau suatu hari mau 'unfreeze', kamu bisa:"
echo "  - Restore /etc/apt/sources.list dari backup"
echo "  - mv /etc/apt/sources.list.d.frozen-... kembali ke /etc/apt/sources.list.d"
echo "  - apt-mark unhold <nama_paket> (untuk paket tertentu saja)"
echo "======================================="
