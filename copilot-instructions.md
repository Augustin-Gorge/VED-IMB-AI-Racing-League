# Copilot Instructions — TORCS Autopilot Project

## General Rules

- ALL code, variables, function names, prints, logs, and comments MUST be written in **English only**
- Keep code simple, explicit, and readable
- Prefer clear logic over clever or complex implementations
- Prefer dynamic formulas over static ones.

---

## Critical Convention — Angles (IMPORTANT)

⚠️ **Angle conventions are INVERTED in this project**

- Positive and negative angles behave opposite to standard mathematical conventions
- Always verify sign usage when working with:
  - steering
  - yaw
  - angle error
  - sensor interpretation

**Never assume standard trigonometric orientation**

---

## Coding Guidelines

- Use descriptive variable names
- Avoid unnecessary abstractions
- Keep functions short and focused
- Avoid duplicates or fonctions that have the same purpose

---

## Physics & Control

- Do NOT change physical constants without justification
- Always validate behavior using telemetry (`telemetry.csv`) and report (`report.md`)
- Any change must be testable and measurable

---

## Telemetry & Debugging

- Use clear and concise debug prints
- Focus on key signals:
  - `target_speed`
  - `speed_x`
  - `trackPos`
  - `state`
- Avoid excessive logging

---

## Development Strategy

- One change at a time
- Test → analyze → iterate
- Stability > speed

---

## What to Avoid

- Hidden magic numbers
- Hardcoded hacks without explanation
- Overfitting to one corner or scenario
- Breaking recovery logic

---

## Expected Mindset

- Think like a control engineer
- Prioritize robustness and predictability
- Every modification must improve measurable performance