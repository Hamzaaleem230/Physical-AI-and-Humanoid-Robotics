// SelectionTooltip.js
import React from "react";
import styles from "./SelectionTooltip.module.css";

export default function SelectionTooltip({ visible, position, onClick }) {
  if (!visible) return null;

  return (
    <div
      className={styles.tooltip}
      style={{ top: position.top, left: position.left }}
      onClick={onClick}
    >
      <span className={styles.emoji}>🤖</span>
      Ask AI
    </div>
  );
}
