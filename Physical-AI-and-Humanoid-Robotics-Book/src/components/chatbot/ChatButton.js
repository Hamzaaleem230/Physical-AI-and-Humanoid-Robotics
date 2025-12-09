import React from 'react';
import styles from './ChatButton.module.css'; // We'll create this CSS module next

function ChatButton({ onClick }) {
  return (
    <button className={styles.chatButton} onClick={onClick}>
      💬
    </button>
  );
}

export default ChatButton;
