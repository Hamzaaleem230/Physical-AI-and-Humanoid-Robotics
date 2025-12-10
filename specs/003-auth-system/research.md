# Research & Decisions: Authentication System

**Branch**: `003-auth-system` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)

This document records the research and key technical decisions made during the planning phase for the Authentication System feature.

## 1. Performance Goal Definition

- **Topic**: Defining specific performance goals for the authentication backend API.
- **Decision**: All authentication-related API endpoints (`/signup`, `/signin`, `/me`, etc.) MUST have a 99th percentile (p99) response time of **less than 500ms**.
- **Rationale**: Authentication is a critical, user-facing interaction. A sub-500ms response time is perceived as nearly instantaneous and is a standard target for high-performance web services. This ensures a smooth and responsive user experience during login and signup.
- **Alternatives Considered**: A looser target (e.g., <1000ms) was rejected as it could lead to noticeable lag. A stricter target (e.g., <200ms) was deemed unnecessary for this application's initial scale and could lead to premature optimization.

## 2. Scalability Planning

- **Topic**: Defining the initial user scale the system should be designed for.
- **Decision**: The system will be designed and provisioned to handle an initial load of up to **10,000 users**.
- **Rationale**: This is a reasonable starting point for a new feature on a specialized educational platform. The chosen serverless technologies (Neon DB, HuggingFace Spaces, Vercel) scale horizontally, so the architecture can support growth beyond this initial target. This number provides a concrete goal for database provisioning and resource allocation.
- **Alternatives Considered**: Planning for 100,000+ users was rejected as over-engineering for the initial launch. Planning for <1,000 users was considered but deemed too short-sighted.

## 3. Docusaurus Global State Integration

- **Topic**: Determining the correct method for integrating a global `AuthProvider` in the Docusaurus application without breaking static site generation (`npm run build`).
- **Decision**: A React Context Provider (`AuthProvider`) will be wrapped around the root of the application using the Docusaurus **`swizzle`** feature on the `Root` component (`src/theme/Root.js`).
- **Rationale**: Docusaurus is a static site generator. Client-side state that depends on browser APIs like `localStorage` must only be rendered on the client. Wrapping the application in `src/theme/Root.js` is the officially documented and recommended method for adding global providers. This component runs in both server-side generation (SSG) and client-side execution, so the provider must be implemented to handle the absence of browser APIs during the build process (e.g., by rendering a fallback or `null`). This ensures the site builds correctly while enabling global client-side state.
- **Alternatives Considered**:
    - **Wrapping the Layout component**: This is another valid Docusaurus approach, but `Root` is more idiomatic for truly global providers that should wrap every single page, including error pages.
    - **Directly modifying `App.js`**: This is not a recommended or stable approach and would be prone to breaking on Docusaurus updates.

## 4. Backend and Frontend Communication

- **Topic**: Defining the communication pattern between the React frontend and the FastAPI backend.
- **Decision**: Communication will be handled via standard RESTful API calls over HTTPS. The `better-auth` library will manage authentication state using **JWTs (JSON Web Tokens)** stored in secure, HTTP-only cookies.
- **Rationale**: This is a standard, secure, and stateless approach for modern web applications. Using HTTP-only cookies for JWTs is a security best practice that mitigates Cross-Site Scripting (XSS) attacks, as the tokens cannot be accessed by client-side JavaScript. The frontend will make authenticated requests, and the browser will automatically include the cookie.
- **Alternatives Considered**: Storing JWTs in `localStorage` was rejected due to its vulnerability to XSS attacks.
