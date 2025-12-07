import os
from pathlib import Path
from typing import List, Dict

def chunk_text(text: str, max_chunk_size: int = 500, overlap: int = 50) -> List[str]:
    """
    Splits text into chunks of a maximum size with a specified overlap.
    This is a basic sentence-aware chunking strategy.
    """
    sentences = text.replace('\n', ' ').split('. ') # Basic sentence splitting
    chunks = []
    current_chunk = []
    current_chunk_len = 0

    for sentence in sentences:
        # Add the period back for complete sentences
        sentence_with_period = sentence + ('.' if not sentence.endswith('.') and not sentence.endswith('?') and not sentence.endswith('!') else '')

        if current_chunk_len + len(sentence_with_period) <= max_chunk_size:
            current_chunk.append(sentence_with_period)
            current_chunk_len += len(sentence_with_period)
        else:
            chunks.append(" ".join(current_chunk).strip())
            # Start new chunk with overlap
            current_chunk = current_chunk[-overlap:] if overlap > 0 else []
            current_chunk_len = sum(len(s) for s in current_chunk) + len(sentence_with_period)
            current_chunk.append(sentence_with_period)
    
    if current_chunk:
        chunks.append(" ".join(current_chunk).strip())
    
    return [chunk for chunk in chunks if chunk] # Filter out empty chunks


def parse_docusaurus_docs(docs_path: Path) -> List[Dict]:
    """
    Parses all .md and .mdx files from the specified Docusaurus docs path.

    Args:
        docs_path: The path to the Docusaurus docs directory.

    Returns:
        A list of dictionaries, where each dictionary contains 'file_path' and 'content'.
    """
    parsed_documents = []
    for root, _, files in os.walk(docs_path):
        for file in files:
            if file.endswith((".md", ".mdx")):
                file_path = Path(root) / file
                try:
                    content = file_path.read_text(encoding="utf-8")
                    parsed_documents.append({"file_path": str(file_path), "content": content})
                except Exception as e:
                    print(f"Error reading file {file_path}: {e}")
    return parsed_documents

if __name__ == "__main__":
    # Assuming this script is run from the project root
    docusaurus_docs_root = Path("../Physical-AI-and-Humanoid-Robotics-Book/docs")
    
    if not docusaurus_docs_root.exists():
        print(f"Error: Docusaurus docs path not found at {docusaurus_docs_root}")
    else:
        print(f"Parsing documents from: {docusaurus_docs_root}")
        documents = parse_docusaurus_docs(docusaurus_docs_root)
        print(f"Found {len(documents)} documents.")
        if documents:
            for doc in documents[:1]: # Print first document for verification
                print(f"--- Document: {doc['file_path']} ---")
                print(f"Original content length: {len(doc['content'])}")
                
                chunks = chunk_text(doc['content'])
                print(f"Generated {len(chunks)} chunks.")
                for i, chunk in enumerate(chunks[:3]): # Print first 3 chunks
                    print(f"--- Chunk {i+1} (length: {len(chunk)}) ---")
                    print(f"{chunk}\n")
