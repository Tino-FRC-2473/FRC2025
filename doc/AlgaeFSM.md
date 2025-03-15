# Algae Remover FSM Spec

```mermaid
---
title: Algae Removal State Diagram
---
stateDiagram-v2

state DEPLOY
state STOW

DEPLOY --> STOW: Algae button pressed
STOW --> DEPLOY: Algae button pressed

```