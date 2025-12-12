// SelectionTooltip.js
import React from "react";
import styles from "./SelectionTooltip.module.css";

export default function SelectionTooltip({ visible, position, onClick }) {
  if (!visible) return null;

  return (
    <div
      className={styles.tooltip}
      style={{
        top: position.top,
        left: position.left,
        position: "absolute",
        pointerEvents: "auto",        // IMPORTANT FIX
        userSelect: "none",           // Prevent accidental text drag
        zIndex: 999999,               // Stronger layer
      }}
      onMouseDown={(e) => e.stopPropagation()}  // Prevents selection collapse
      onClick={onClick}                          // Triggers chatbot + auto-send
    >
      <span className={styles.emoji}>🤖</span>
      Ask AI
    </div>
  );
}
