# Exclusive use of the Loom hosts while a script programs their cards or measures (the U280s and V80s are shared
# with other users). Sourced by setup_loom_hosts.sh, examples/loom_switch/regress.sh, test_build.sh and the
# experiment scripts:
#
#   source "$REPO/scripts/fpga/hosthog_loom.sh"
#   hog_claim <minutes> <host>... || exit 1    # sudo hosthog claim --exclusive <minutes>min loom; sudo hosthog hog
#   ...                                         # released on exit (sudo hosthog release), or at the claim's timeout
#
# hog_claim fails if someone else holds an exclusive claim on a host (hosthog status says who); then it releases the
# hosts it already took. A script started under a claim (LOOM_HOG exported by its parent) neither claims nor releases.
# The hog locks other users out of ssh and pauses the hosts' auto-upgrade/auto-reboot timers; hosthog's maintenance
# job undoes both when the claim times out, so pick <minutes> above the script's run time.

HOG_HOSTS=""

hog_release() {
    local h
    for h in $HOG_HOSTS; do
        timeout 60 ssh -o ConnectTimeout=15 -n "$h.dos.cit.tum.de" "sudo hosthog release" > /dev/null 2>&1 \
            && echo "hosthog: released $h" || echo "hosthog: release on $h failed (the claim times out by itself)"
    done
    HOG_HOSTS=""
}

hog_claim() {
    local min=$1 h; shift
    if [ -n "${LOOM_HOG:-}" ]; then echo "hosthog: under the parent's claim ($LOOM_HOG)"; return 0; fi
    for h in "$@"; do
        if timeout 60 ssh -o ConnectTimeout=15 -n "$h.dos.cit.tum.de" \
               "sudo hosthog claim --exclusive ${min}min loom > /dev/null && sudo hosthog hog > /dev/null"; then
            HOG_HOSTS="$HOG_HOSTS $h"
        else
            echo "hosthog: no exclusive claim on $h:"
            timeout 60 ssh -o ConnectTimeout=15 -n "$h.dos.cit.tum.de" "hosthog status" 2>&1 | sed 's/^/   /'
            hog_release
            return 1
        fi
    done
    export LOOM_HOG="$(echo $HOG_HOSTS) until $(date -d "+$min min" +%H:%M)"
    trap hog_release EXIT
    echo "hosthog: claimed $LOOM_HOG"
}
