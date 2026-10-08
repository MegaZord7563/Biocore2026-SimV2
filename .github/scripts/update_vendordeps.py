"""Open one pull request per outdated WPILib vendordep.

Reads every JSON in ./vendordeps, compares its version with the one published at
its `jsonUrl`, and opens a PR when a newer version exists for the same FRC year.

Environment variables:
  GH_TOKEN     token used by the `gh` CLI (required)
  BASE_BRANCH  branch PRs target (default: main)
  FRC_YEAR     only accept updates whose `frcYear` matches (e.g. "2027"); optional
"""
import json
import os
import re
import subprocess
import sys

import requests
from packaging import version

VENDORDEP_DIR = "./vendordeps"
BASE_BRANCH = os.environ.get("BASE_BRANCH", "main")
FRC_YEAR = os.environ.get("FRC_YEAR")


def run(*cmd, check=True, capture=False):
    return subprocess.run(cmd, check=check, text=True, capture_output=capture)


def slug(text):
    """Make a string safe for branch names."""
    return re.sub(r"[^A-Za-z0-9._-]+", "-", str(text)).strip("-")


def parse(v):
    try:
        return version.parse(v)
    except (version.InvalidVersion, TypeError):
        return None


def already_handled(branch):
    """True if a PR (open, merged or closed) or a remote branch already exists."""
    prs = run("gh", "pr", "list", "--head", branch, "--state", "all",
              "--json", "number", capture=True).stdout
    if json.loads(prs):
        return True
    remote = run("git", "ls-remote", "--heads", "origin", branch,
                 capture=True).stdout
    return bool(remote.strip())


def process(file):
    file_path = os.path.join(VENDORDEP_DIR, file)
    with open(file_path) as f:
        local = json.load(f)

    json_url = local.get("jsonUrl")
    if not json_url:
        print(f"{file}: no 'jsonUrl', skipped")
        return

    resp = requests.get(json_url, timeout=30)
    resp.raise_for_status()
    remote = resp.json()

    name = remote.get("name") or local.get("name") or file

    remote_year = str(remote.get("frcYear", ""))
    if FRC_YEAR and remote_year and remote_year != FRC_YEAR:
        print(f"{name}: frcYear {remote_year} != {FRC_YEAR}, skipped")
        return

    local_v, remote_v = parse(local.get("version")), parse(remote.get("version"))
    if local_v is None or remote_v is None:
        raise ValueError(f"unrecognised version format for {name}")
    if remote_v <= local_v:
        print(f"{name}: up to date ({local['version']})")
        return

    old_v, new_v = local["version"], remote["version"]
    branch = f"update-{slug(name)}-{slug(new_v)}"
    if already_handled(branch):
        print(f"{name}: branch/PR {branch} already exists, skipped")
        return

    # Never let a remote 'fileName' escape the vendordeps folder.
    new_file = os.path.basename(remote.get("fileName") or file)
    new_path = os.path.join(VENDORDEP_DIR, new_file)

    run("git", "checkout", BASE_BRANCH)
    run("git", "checkout", "-b", branch)

    # Write the upstream text as-is to avoid noisy reformatting diffs.
    text = resp.text if resp.text.endswith("\n") else resp.text + "\n"
    with open(new_path, "w") as f:
        f.write(text)
    if new_path != file_path:
        print(f"Renaming {file} -> {new_file}")
        os.remove(file_path)

    msg = f"Bump {name} from {old_v} to {new_v}"
    print(msg)
    run("git", "add", "-A", VENDORDEP_DIR)
    run("git", "commit", "-m", msg)
    run("git", "push", "-u", "origin", branch)
    run("gh", "pr", "create",
        "--title", msg,
        "--base", BASE_BRANCH,
        "--head", branch,
        "--body", (f"### Automated update for {name}\n"
                   f"- **Old version:** {old_v}\n"
                   f"- **New version:** {new_v}\n"
                   f"- **Source:** {json_url}\n\n"
                   "Check the build and test on a robot or in simulation "
                   "before merging."))


def main():
    failures = 0
    for file in sorted(os.listdir(VENDORDEP_DIR)):
        if not file.endswith(".json"):
            continue
        try:
            process(file)
        except Exception as e:  # keep going so one bad vendordep doesn't block the rest
            failures += 1
            print(f"ERROR in {file}: {e}")
        finally:
            run("git", "checkout", "-f", BASE_BRANCH, check=False)
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()