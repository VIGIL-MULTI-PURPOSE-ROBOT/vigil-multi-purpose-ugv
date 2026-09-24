#!/usr/bin/env bash
# Publish the three VIGIL projects + the master launcher to GitHub, branch main of
#   https://github.com/VIGIL-MULTI-PURPOSE-ROBOT/agri-ugv
#
#   bash ~/Documents/vigil/publish_to_github.sh
#
# Repository layout (the same one ~/Documents/robot already has, plus two new folders):
#   ros2_ws/                 <- ~/Documents/robot/ros2_ws                  (Agriculture)
#   vigil_rough_terrain_ws/  <- ~/Documents/robot/vigil_rough_terrain_ws   (Rock Terrain)
#   military_world/          <- ~/Documents/military_world                 (Military SAR)   NEW
#   vigil/                   <- ~/Documents/vigil                          (master launcher) NEW
#
# How: a separate clone in ~/Documents/.vigil_publish is filled from the real folders (nothing on
# your PC is moved, and ~/Documents/robot is not touched), then committed and pushed. Files are
# added or updated only; nothing already on main is deleted. Build output (build/ install/ log/
# run/ generated/ ...) is never uploaded. You see the list and confirm before anything is pushed.
# Pushing uses YOUR GitHub login (the same one that pushed ~/Documents/robot before).
set -euo pipefail

REPO_URL="${VIGIL_REPO_URL:-https://github.com/VIGIL-MULTI-PURPOSE-ROBOT/agri-ugv.git}"
BRANCH="${VIGIL_BRANCH:-main}"
D="$HOME/Documents"
WORK="$D/.vigil_publish"

command -v git   >/dev/null || { echo "git is not installed:  sudo apt install git"; exit 1; }
command -v rsync >/dev/null || { echo "rsync is not installed:  sudo apt install rsync"; exit 1; }
if [[ -z "$(git config --global user.name || true)" || -z "$(git config --global user.email || true)" ]]; then
  echo 'Set your git name/e-mail once:  git config --global user.name "Your Name" && git config --global user.email you@example.com'
  exit 1
fi

# source folder | repo folder | rsync excludes (each project's own build output)
COMMON=(--exclude=__pycache__/ --exclude=.pytest_cache/ --exclude='*.pyc')
declare -a SRC=("$D/robot/ros2_ws" "$D/robot/vigil_rough_terrain_ws" "$D/military_world" "$D/vigil")
declare -a DST=("ros2_ws" "vigil_rough_terrain_ws" "military_world" "vigil")
EXC_0=(--exclude=/build/ --exclude=/install/ --exclude=/log/ --exclude=/run/ --exclude=/.migration-backup/)
EXC_1=(--exclude=/build/ --exclude=/install/ --exclude=/log/ --exclude=/run/)
EXC_2=(--exclude=/build/ --exclude='/Claude outputs/' --exclude='*.blend1' --exclude='*.log'
       --exclude=/sar_ws/build/ --exclude=/sar_ws/install/ --exclude=/sar_ws/log/
       --exclude=/sar_ws/generated/ --exclude=/sar_ws/diagnosis/ --exclude=/sar_ws/test_results/)
EXC_3=(--exclude=/agriculture --exclude=/rock_terrain --exclude=/military_sar --exclude=/REPO_README.md)   # links; front page

for s in "${SRC[@]}"; do [[ -d "$s" ]] || { echo "missing: $s"; exit 1; }; done

echo "== 1/5 getting $BRANCH of $REPO_URL"
if [[ -d "$WORK/.git" ]]; then
  git -C "$WORK" fetch origin
  git -C "$WORK" checkout -q "$BRANCH"
  git -C "$WORK" reset -q --hard "origin/$BRANCH"
else
  rm -rf "$WORK"
  git clone --branch "$BRANCH" "$REPO_URL" "$WORK"
fi
touch "$D/.vigil_publish/COLCON_IGNORE" 2>/dev/null || true
grep -qx 'COLCON_IGNORE' "$WORK/.git/info/exclude" 2>/dev/null || echo 'COLCON_IGNORE' >> "$WORK/.git/info/exclude"

echo "== 2/5 copying the projects into the clone (add / update only)"
for i in 0 1 2 3; do
  ex="EXC_$i[@]"
  mkdir -p "$WORK/${DST[$i]}"
  rsync -a "${COMMON[@]}" "${!ex}" "${SRC[$i]}/" "$WORK/${DST[$i]}/"
  echo "   ${SRC[$i]}  ->  ${DST[$i]}/"
done
# the repository's front page: ~/Documents/vigil/REPO_README.md -> README.md
[[ -f "$D/vigil/REPO_README.md" ]] && cp "$D/vigil/REPO_README.md" "$WORK/README.md" && echo "   vigil/REPO_README.md  ->  README.md"

echo "== 3/5 checks"
cd "$WORK"
git add -A
big=$(git diff --cached --name-only -z | xargs -0 -r du -k 2>/dev/null | awk '$1 > 97280 {print $2}')
if [[ -n "$big" ]]; then echo "ABORT: files over GitHub's 100 MB limit:"; echo "$big"; exit 1; fi
warn=$(git diff --cached --name-only -z | xargs -0 -r du -k 2>/dev/null | awk '$1 > 51200 {printf "   %s (%d MB)\n", $2, $1/1024}')
[[ -n "$warn" ]] && { echo "   large files (allowed, GitHub warns above 50 MB):"; echo "$warn"; }
secret=$(git diff --cached --name-only -z | xargs -0 -r grep -l -I -E 'ghp_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,}|BEGIN (RSA|OPENSSH|EC) PRIVATE KEY|AKIA[0-9A-Z]{16}' 2>/dev/null || true)
if [[ -n "$secret" ]]; then echo "ABORT: these files look like they contain a key/token:"; echo "$secret"; exit 1; fi

if git diff --cached --quiet; then echo "Nothing new: GitHub $BRANCH already has all of it."; exit 0; fi
n_new=$(git diff --cached --name-only --diff-filter=A | wc -l)
n_mod=$(git diff --cached --name-only --diff-filter=M | wc -l)
size=$(git diff --cached --name-only -z | xargs -0 -r du -ck 2>/dev/null | tail -1 | awk '{printf "%.1f", $1/1024}')
echo "   $n_new new files, $n_mod changed files, ~${size} MB, per folder:"
git diff --cached --name-only | cut -d/ -f1 | sort | uniq -c | sed 's/^/     /'

echo "== 4/5 commit"
read -r -p "Push these to $BRANCH of ${REPO_URL}? [y/N] " ok
[[ "$ok" == y || "$ok" == Y ]] || { echo "Nothing pushed (the prepared clone is in $WORK)."; exit 0; }
git commit -q -F - <<'MSG'
Update VIGIL simulation projects

- military_world/: military search-and-rescue world and the vigil_sar ROS 2 workspace (sar_ws)
- vigil/: master launcher menu that starts ONE of the three independent projects
- ros2_ws/ (agriculture) and vigil_rough_terrain_ws/ (rock terrain): current state
Build output (build/, install/, log/, generated/) is not included.
MSG

echo "== 5/5 push"
git push origin "HEAD:$BRANCH"
echo "Done: https://github.com/VIGIL-MULTI-PURPOSE-ROBOT/agri-ugv/tree/$BRANCH"
