# Maintaining the presentation

## Demonstration video

The README links to the published VIGIL demonstration:

```markdown
[demo-video]: https://youtu.be/WZutmP8n8Oc?si=ThVBvrjVecI3OAJ3
```

To change the video later, replace only the URL on this reference-definition line in the root `README.md`. The “Watch the VIGIL demonstration” link uses it.

## Update results

Keep historical results separate from fresh measurements. For each run, record the source commit, launch command, scenario, pose source, outcome and limitations. Save small status snapshots under `docs/validation/` and captures under `docs/media/`; large videos should be linked externally.

A screenshot is evidence of the visible state, not proof of a trained model, visual localization, collision-free operation or general terrain performance.

## Repository organization

Documentation was reorganized without moving the working ROS workspaces. The old root guide is preserved in `docs/operations.md`. Source folders can be consolidated later, but only with asset-path and launch testing. Do not delete working code to simplify the GitHub front page.
