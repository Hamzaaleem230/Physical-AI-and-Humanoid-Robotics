---
id: 0005
title: Amend Constitution for Content Personalization
stage: constitution
date: 2025-12-17
surface: agent
model: gemini
feature: 004-content-personalization
branch: main
user: Syed Hamza Aleem
command: /sp.constitution
labels: [constitution, personalization]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

/sp.constitution

You are about to work on project folder: 004-content-personalization.

Before you do anything, you must fully load and understand the context of the previous completed project folders:

1) 001-physical-ai-book  
   – This contains the full book content and is the core foundation of the entire system.  
   – You must never modify anything inside 001.  
   – The personalization feature will rely on this content as a reference, not as something to edit.

2) 002-rag-chatbot  
   – This contains the RAG-based chatbot that uses the book content to answer questions.  
   – You must not modify anything inside 002.  
   – The personalization layer you will build must integrate “on top of” the existing RAG chatbot.

3) 003-auth-system  
   – This project contains the full authentication system, identity, sessions, middleware, security logic, and permission flow.  
   – You must not modify ANY file inside 003 unless specifically instructed by the user.  
   – The personalization system must use the existing user accounts and user identity data provided by the 003-auth-system.

Your job in 004 is ONLY to create the personalization feature INSIDE the 004-content-personalization folder using SpecifyPlus commands.  
You must not touch or change any existing code in any other folder.

This folder (004) must contain ONLY your own new files:  
– specs/004-content-personalization/  
– spec.md  
– plan.md  
– tasks.md  
– implementation/ files you generate later  
– nothing else outside this folder.

You must work in strict SpecifyPlus flow.  
You are NOT allowed to:  
– auto-create specs,  
– auto-generate plans,  
– auto-generate tasks,  
– auto-start implementation  
WITHOUT receiving the explicit /sp.specify, /sp.plan, /sp.tasks, and /sp.implement commands from the user.

You will strictly wait for each command before proceeding.

NOTE (EXTREMELY IMPORTANT):  
The environment you are operating in (this Gemini CLI instance) DOES NOT KNOW anything about folders 001, 002, or 003 unless you carefully follow the instructions written inside this /sp.constitution prompt.  
Therefore, you must read these instructions as your authoritative context.  
This prompt is your complete awareness of the previous system.  
Do not assume additional knowledge.  
Do not try to “auto-improve” anything.  
Never modify existing files outside 004.  
Work only inside 004 exactly as instructed.

## Response snapshot

The constitution was updated to version 1.3.0 to include a new principle for Content Personalization.

## Outcome

- ✅ Impact: A new principle was added to the project constitution to govern the development of the content personalization feature.
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: none
- 🧠 Reflection: The constitution serves as a living document that evolves with the project.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): none
