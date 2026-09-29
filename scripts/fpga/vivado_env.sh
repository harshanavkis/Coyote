# Vivado environment for scripts/fpga/*.sh, sourced by them (not run).
#
#   vivado_version_for <device>   -> 2023.2 (u280|u55c|u250) or 2025.1 (v80)
#   vivado_env <version>          -> a shell snippet that sets that version up
#                                    INSIDE xilinx-shell; prepend it to the
#                                    command given to `xilinx-shell -c`
#
# xilinx-shell starts from a clean environment with Vivado 2023.2 set up.
# 2025.1 needs that environment cleared, its own settings sourced (Vivado and
# Vitis, for vitis-run) and ncurses 6 (libtinfo.so.6). Every version gets
# LM_LICENSE_FILE for the CMAC IP. UltraScale+ uses 2023.2 because Coyote's
# U280/U55C/U250 static checkpoints are Vivado 2022.1 files, which 2025.1
# cannot open; the V80's checkpoint is 2024.2, and Coyote requires >= 2024.2.

VIVADO_LICENSE=/share/xilinx/Xilinx.lic

vivado_version_for() {
    case $1 in
        u280|u55c|u250) echo 2023.2 ;;
        v80)            echo 2025.1 ;;
        *) echo "unknown device '$1'" >&2; return 1 ;;
    esac
}

vivado_env() {
    local version=$1 env="" ncurses
    [ -r "$VIVADO_LICENSE" ] || { echo "license file $VIVADO_LICENSE not readable" >&2; return 1; }
    if [ "$version" = 2025.1 ]; then
        ncurses=$(nix-build '<nixpkgs>' -A ncurses --no-out-link 2>/dev/null | tail -1)
        [ -e "$ncurses/lib/libtinfo.so.6" ] || { echo "could not resolve ncurses 6 via nix" >&2; return 1; }
        env="unset XILINX_VIVADO XILINX_HLS XILINX_VITIS XILINX_SDX XILINX_PATH MYVIVADO MYXILINX RDI_BINROOT RDI_APPROOT RDI_BASEROOT RDI_INSTALLROOT RDI_INSTALLVER RDI_BASELINE RDI_PATCHROOT RDI_SHARED_DATA HDI_APPROOT _RDI_SETENV_RUN; \
PATH=\$(printf '%s' \"\$PATH\" | tr ':' '\n' | awk '\$0 !~ \"^/share/xilinx/\"' | paste -sd: -); export PATH; \
source /share/xilinx/2025.1/Vivado/.settings64-Vivado.sh >/dev/null 2>&1; \
source /share/xilinx/2025.1/Vitis/.settings64-Vitis.sh >/dev/null 2>&1; \
export XILINX_LOCAL_USER_DATA=no; \
export LD_LIBRARY_PATH=$ncurses/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH};"
    fi
    echo "$env export LM_LICENSE_FILE=$VIVADO_LICENSE; export XILINX_JOBS=256; export TERM=\${TERM:-xterm};"
}

# Default PCIe addresses of the cards, per host
default_bdf() {   # <u280|v80>
    case "$(hostname)-$1" in
        clara-u280|amy-u280) echo 0000:e1:00.0 ;;
        rose-u280)           echo 0000:c1:00.0 ;;
        clara-v80)           echo 0000:81:00.0 ;;
        rose-v80)            echo 0000:61:00.0 ;;
        *) echo "no default $1 BDF for $(hostname); pass one" >&2; return 1 ;;
    esac
}
