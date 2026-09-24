#!/usr/bin/env bash
# One-time fix of VIGIL-MULTI-PURPOSE-ROBOT/agri-ugv (branch main):
#   1. removes the "Co-Authored-By: Claude ..." and "Claude-Session: ..." lines from the commit
#      messages (that is what shows "claude" next to your name on GitHub)
#   2. replaces the old agriculture-only README.md with ~/Documents/vigil/REPO_README.md
# Then force-pushes main (only if nobody pushed in between). Shows everything and asks first.
#
#   bash ~/Documents/vigil/fix_github.sh
set -euo pipefail
REPO_URL="${VIGIL_REPO_URL:-https://github.com/VIGIL-MULTI-PURPOSE-ROBOT/agri-ugv.git}"
BRANCH=main
D="$HOME/Documents"
WORK="$D/.vigil_publish"
README="$D/vigil/REPO_README.md"
[[ -f "$README" ]] || { echo "missing $README"; exit 1; }

if [[ -d "$WORK/.git" ]]; then
  git -C "$WORK" fetch -q origin
  git -C "$WORK" checkout -q "$BRANCH"
  git -C "$WORK" reset -q --hard "origin/$BRANCH"
else
  git clone -q --branch "$BRANCH" "$REPO_URL" "$WORK"
fi
cd "$WORK"
REMOTE_SHA=$(git rev-parse "origin/$BRANCH")

echo "== commits that mention Claude:"
bad=$(git log --format='%H' --grep='[Cc]laude' "$BRANCH" || true)
if [[ -n "$bad" ]]; then
  git log --format='   %h %an: %s' --grep='[Cc]laude' "$BRANCH"
  git log --format='%B' --grep='[Cc]laude' "$BRANCH" | grep -i 'claude' | sed 's/^/      line: /'
  export FILTER_BRANCH_SQUELCH_WARNING=1
  git filter-branch -f --msg-filter \
    "sed -E '/^(Co-Authored-By: *Claude|Claude-Session:)/Id' | sed -e :a -e '/^\n*\$/{\$d;N;ba' -e '}'" \
    -- "$BRANCH" >/dev/null
  rm -rf .git/refs/original
  git log --format='%B' "$BRANCH" | grep -qi 'claude' && { echo "ABORT: a commit message still mentions Claude"; exit 1; }
  echo "   -> removed from the commit messages"
else
  echo "   none"
fi

echo "== README.md"
cp "$README" README.md
if git diff --quiet -- README.md; then
  echo "   already up to date"
else
  git add README.md
  git commit -q -m "Update README for the three VIGIL simulation projects and the launcher"
  echo "   -> new README committed"
fi

hits=$(git grep -il 'claude' -- . ':!*.png' ':!*.jpg' ':!*.stl' ':!*.glb' ':!*.blend' || true)
if [[ -n "$hits" ]]; then
  echo "== note: these files in the repo contain the word 'claude' (not changed, check them yourself):"
  echo "$hits" | sed 's/^/   /'
fi

if [[ "$(git rev-parse HEAD)" == "$REMOTE_SHA" ]]; then echo "Nothing to push."; exit 0; fi
echo
git log --format='   %h %an: %s' "$BRANCH" | head -5
read -r -p "Force-push this history to $BRANCH (replaces the commits shown on GitHub)? [y/N] " ok
[[ "$ok" == y || "$ok" == Y ]] || { echo "Nothing pushed."; exit 0; }
git push --force-with-lease="$BRANCH:$REMOTE_SHA" origin "HEAD:$BRANCH"
echo "Done: https://github.com/VIGIL-MULTI-PURPOSE-ROBOT/agri-ugv"
