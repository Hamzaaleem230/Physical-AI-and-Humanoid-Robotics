# Research: RAG Chatbot Integration

**Date**: 2025-12-07
**Feature**: RAG Chatbot Integration

This document outlines the key technical decisions made during the planning phase for the RAG chatbot.

## 1. Frontend Integration Method

**Task**: Research the best method for injecting a global React component (the chatbot) into the existing Docusaurus application.

**Options Considered**:
1.  **Docusaurus Plugin**: Create a custom Docusaurus plugin that injects the component. This is clean but adds complexity and boilerplate.
2.  **Swizzling Docusaurus Theme Components**: Docusaurus allows developers to override any theme component by "swizzling" it. This provides a direct way to add custom wrappers or components. Common targets are the `Layout` or `Root` components.
3.  **Manual Script Injection**: Use a `post-build` script to manually inject the chatbot's JavaScript into the final HTML output. This is brittle and not recommended.

**Decision**:
We will use **Docusaurus Swizzling** to wrap the main `Layout` component.

**Rationale**:
- **Standard Practice**: Swizzling is the officially recommended method by the Docusaurus team for this exact use case (adding global components like cookie banners or chat widgets).
- **Simplicity and Control**: It is simpler than writing a full plugin but far more robust and integrated than manual script injection.
- **Maintainability**: The override is clearly located in the `src/theme` directory, making it easy to find and update. It also benefits from the Docusaurus build process, including hot-reloading in development.

## 2. Backend Communication and CORS

**Task**: Determine the strategy for communication between the Docusaurus frontend and the FastAPI backend, specifically addressing Cross-Origin Resource Sharing (CORS).

**Options Considered**:
1.  **Proxy via Docusaurus**: Configure the Docusaurus development server to proxy API requests to the backend. This is a common pattern in local development but doesn't solve the production issue.
2.  **Backend CORS Middleware**: Configure the FastAPI backend to accept requests from the frontend's domain.
3.  **API Gateway**: Place both services behind an API Gateway that handles routing and CORS. This adds significant infrastructure complexity.

**Decision**:
The **FastAPI backend will use `CORSMiddleware`**.

**Rationale**:
- **Simplicity**: `CORSMiddleware` is a standard part of the FastAPI ecosystem and is trivial to configure.
- **Effectiveness**: It directly solves the CORS problem for both development and production environments by creating a clear policy of which origins are allowed to communicate with the API.
- **Flexibility**: The list of allowed origins can be configured via environment variables, allowing the same codebase to be deployed to different environments (e.g., local, staging, production) without code changes. The frontend will simply make requests to the backend URL specified in its own environment configuration.
