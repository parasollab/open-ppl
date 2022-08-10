---
author:
- James Motes
title: "**Procedures for PPL Development**"
---

This document outlines the procedures for PPL development.

# Check-out Procedure

All new development should occur on a new branch. New branches should be spawned
from the main branch (release-stage-one). This can be done with the following
command:
```
git checkout -b myNewBranch mainBranch
```
i.e.
```
git checkout -b basic-prm release-stage-one
```

# Check-in Procedure

New commits should be pushed regularly to your development branch. You should
never push directly to the main branch (release-stage-one).

To merge new features into the main branch, you must submit a merge request on
gitlab. You should assign someone else other than yourself as a reviewer.
Reviews should cover both coding standards and logic. Once all comments have
been addressed, the branch can be merged into the main branch (release-stage-one).

At a later date, we will include instructions for testing code before checking it in.
