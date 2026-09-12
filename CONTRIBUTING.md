<div align="center">

<img src="https://github.com/user-attachments/assets/25f6ee68-5a92-42ed-a6e0-bf9e7c493577" alt="Megazord7563" height="90em" />

# Contributing to Biocore2026‑SimV2

<sub>Team 7563 Megazord — programming team guidelines</sub>

<br />

<img src="https://img.shields.io/badge/team-FRC_7563-blue?style=flat-square" />
<img src="https://img.shields.io/badge/code_style-Google_Java_Format-2ea043?style=flat-square" />
<img src="https://img.shields.io/badge/PRs-welcome-8957e5?style=flat-square" />

<br /><br />

<a href="#-branching-strategy">Branching</a> ·
<a href="#-version-control-workflow">Workflow</a> ·
<a href="#-repository-organization">Organization</a> ·
<a href="#-team-collaboration">Collaboration</a> ·
<a href="#-testing-on-the-robot">Testing</a>

</div>

<br />

Thanks for contributing to the Biocore season codebase. This document sets the ground rules the programming team follows so the robot code stays stable, reviewable and safe to deploy at competition.

<br />

## 🌿 Branching Strategy

<table>
<tr>
<td width="30%" valign="top"><b>🟢 Main / Master</b></td>
<td>Always represents stable, deployable code known to work on the robot. Only merge well‑tested and reviewed changes here.</td>
</tr>
<tr>
<td width="30%" valign="top"><b>🌱 Feature branches</b></td>
<td>Create a dedicated branch off <code>main</code> for every new feature, bug fix or significant change. This isolates your work and prevents breaking the main codebase.</td>
</tr>
<tr>
<td width="30%" valign="top"><b>🔗 Issue branches</b></td>
<td>Link feature branches to a specific GitHub Issue or task. This keeps clear context for every change.</td>
</tr>
</table>

<sub>Suggested naming: <code>feature/swerve-autonomous</code>, <code>fix/turret-soft-limits</code>, <code>issue-42-intake-current-spike</code>.</sub>

<br />

## 🔁 Version Control Workflow

<table>
<tr>
<td width="30%" valign="top"><b>📝 Regular commits</b></td>
<td>Commit frequently with clear, descriptive messages. This builds a detailed history and makes reverting easy when something breaks on the field.</td>
</tr>
<tr>
<td width="30%" valign="top"><b>🔀 Pull requests</b></td>
<td>All changes land on <code>main</code> through a PR — never push directly. This gives teammates a chance to review, comment and catch issues before they reach the robot.</td>
</tr>
<tr>
<td width="30%" valign="top"><b>👀 Code review</b></td>
<td>Focus reviews on functionality, style, and adherence to FRC best practices and team standards — not just "does it compile".</td>
</tr>
<tr>
<td width="30%" valign="top"><b>🤖 Testing</b></td>
<td>Deploy and thoroughly test feature‑branch code on the robot (or in sim, for drivetrain/logic changes) <i>before</i> merging into <code>main</code>.</td>
</tr>
</table>

<br />

## 📁 Repository Organization

- **Clear folder structure** — keep code organized under `src/main/java/br/megazord/frc7563/`, grouping mechanisms under `subsystems/<name>/` the way `subsystems/swerve/` is structured today.
- **Constants** — robot‑wide numeric/boolean constants belong in `Constants.java` only. Don't scatter magic numbers across subsystem files.
- **README.md** — keep the setup, build and deploy instructions in the main [README](./README.md) current as the environment changes.
- **Documentation** — comment key algorithms, PID tuning notes and hardware‑interface quirks inline; future teammates (and future you) will need the context.

<br />

## 🤝 Team Collaboration

<table>
<tr>
<td width="30%" valign="top"><b>💬 Communication</b></td>
<td>Keep the team in the loop on code changes, progress and any issues encountered — especially anything that affects drivetrain behavior or safety.</td>
</tr>
<tr>
<td width="30%" valign="top"><b>⚔️ Conflict resolution</b></td>
<td>Address merge conflicts promptly and collaboratively — don't let a stale branch rot.</td>
</tr>
<tr>
<td width="30%" valign="top"><b>🎓 Knowledge sharing</b></td>
<td>Mentor newer programmers, especially around WPILib command‑based patterns and the IO/sim layering used in <code>subsystems/swerve</code>.</td>
</tr>
</table>

<br />

## 🧪 Testing on the Robot

1. Run `./gradlew build` locally — a red build blocks the PR (see the [build workflow](./.github/workflows/build.yml)).
2. Validate drivetrain and control logic in simulation first with `./gradlew simulateJava` when hardware access is limited.
3. Deploy the feature branch to the practice bot and confirm current limits, PID gains and soft limits behave as expected before requesting review.
4. Only merge to `main` once the change has been reviewed **and** run on real hardware (or sim, for non‑mechanism changes).

<br />

<div align="center">
<sub>Questions? Reach out on the team's programming channel before opening a PR.</sub>
</div>