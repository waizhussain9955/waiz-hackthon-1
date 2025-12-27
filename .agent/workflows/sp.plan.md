---
description: Planning workflow for new features or content additions to the Physical AI textbook
---

# /sp.plan - Feature & Content Planning Workflow

Use this workflow when adding new features, lessons, or capabilities to the Physical AI & Humanoid Robotics textbook.

## Step 1: Define Scope
- What is the new feature/content being added?
- Which module(s) does it affect?
- What are the learning objectives?

## Step 2: Check Dependencies
- Review `/sp.constitution` for alignment with core principles
- Review `/sp.specify` for formatting standards
- Identify prerequisite knowledge from existing modules

## Step 3: Create Implementation Plan
Write `implementation_plan.md` artifact with:
- [ ] Goal description and background context
- [ ] User review items (breaking changes, design decisions)
- [ ] Proposed changes grouped by component
- [ ] Verification plan (tests, manual checks)

## Step 4: Get Approval
Request user review of implementation plan before proceeding.

## Step 5: Execute Plan
// turbo-all
- Create/modify markdown files with proper frontmatter
- Include Mermaid diagrams for architecture
- Add Python code examples with syntax highlighting
- Use admonitions (:::tip, :::danger) for important notes

## Step 6: Verify Build
```bash
cd my-website
npm run build
```

## Step 7: Update Docs
- Update `intro.md` if adding new modules
- Update `_category_.json` files as needed
- Create walkthrough artifact documenting changes

## Formatting Reminders
- Frontmatter: `sidebar_label`, `sidebar_position`
- Code blocks: Use ```python or ```cpp
- Diagrams: Use ```mermaid for flowcharts
- Warnings: Use :::danger for hardware safety
