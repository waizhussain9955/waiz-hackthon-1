# Project History: 001 - Initial Setup and Curriculum Restructure

**Date:** December 27, 2025
**Context:** 
The project "AI-Native Textbook: Physical AI & Humanoid Robotics" was initiated to create a published Docusaurus book with a sophisticated RAG chatbot. The initial goal was to restructure the curriculum into a 4-module ROS 2-focused framework and implement the basic Docusaurus site.

**Actions Taken:**
1.  **Curriculum Restructure:**
    *   Removed original 5-chapter structure.
    *   Created 4-module framework:
        *   Module 01: ROS 2 Foundation
        *   Module 02: Digital Twin (Gazebo/Unity)
        *   Module 03: AI-Robot Brain (VLA, NVIDIA Isaac)
        *   Module 04: Capstone Project
    *   Updated `intro.md` and `docusaurus.config.ts` to reflect this new structure.
2.  **Docusaurus Configuration:**
    *   Finalized site branding and deployment settings.
    *   Verified build with `npm run build`.

**Files Affected:**
*   `my-website/docs/` (Directory structure)
*   `my-website/docusaurus.config.ts`
*   `my-website/docs/intro.md`

**Reasoning:**
The restructure aligns the content with standard robotics curriculum flows, emphasizing ROS 2 as the middleware layer early on.

**Outcomes:**
*   Successful Docusaurus build.
*   Site running locally on port 3005.
