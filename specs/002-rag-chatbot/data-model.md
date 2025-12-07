# Data Models: RAG Chatbot

**Date**: 2025-12-07
**Feature**: RAG Chatbot Integration

This document defines the key data entities for the RAG chatbot feature.

## 1. Content Chunk

Represents a piece of text extracted from the source documentation that has been prepared for embedding and storage in the vector database.

- **`id`**: `UUID` (Primary Key) - A unique identifier for the chunk.
- **`source_file`**: `string` - The original file path from which the chunk was extracted (e.g., `docs/ros2-nervous-system/what-is-physical-ai.md`).
- **`content`**: `string` - The actual text content of the chunk.
- **`embedding`**: `List[float]` - The numerical vector representation of the content.

**Validation Rules**:
- `content` must not be empty.
- `embedding` vector must have the expected dimensionality for the chosen model.

## 2. Chat Message

Represents a single message sent by either the user or the chatbot assistant within a conversation.

- **`role`**: `Enum['user', 'assistant']` (Required) - Indicates the sender of the message.
- **`content`**: `string` (Required) - The text of the message.
- **`timestamp`**: `datetime` (Required) - The time at which the message was sent.

**Validation Rules**:
- `role` must be either 'user' or 'assistant'.
- `content` must not be empty.

## 3. Chat Session

Represents a single, continuous conversation between a user and the chatbot. This will primarily be managed on the client-side for now, but a server-side representation is useful for potential future features like persisted chat history.

- **`session_id`**: `UUID` (Primary Key) - A unique identifier for the session.
- **`history`**: `List[ChatMessage]` - An ordered list of messages that make up the conversation.
- **`created_at`**: `datetime` - The timestamp when the session was initiated.

## 4. User Account (Optional)

Represents a registered user of the chatbot. This model is optional and would only be implemented if authentication is added.

- **`id`**: `UUID` (Primary Key)
- **`email`**: `string` (Unique)
- **`hashed_password`**: `string`
- **`created_at`**: `datetime`
- **`updated_at`**: `datetime`

**Relationships**:
- A `User Account` could have many `Chat Session`s.
