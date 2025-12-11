import React, { useState } from "react";
import styles from "./AuthExpandableNotice.module.css";

export default function AuthExpandableNotice() {
  const [expanded, setExpanded] = useState(false);

  const shortText =
    "⚠️ To sign in, you must allow third-party cookies in your browser.";

  const fullText = `
Chrome → Settings → Privacy & Security → Third-party cookies → Allow
Or add exception:
https://[*.]physical-ai-and-humanoid-robotics-ebon.vercel.app
and reload the page.
This is required because the authentication system uses secure cookies.
  `;

  return (
    <div className={`${styles.noticeModal} ${expanded ? styles.expanded : ""}`}>
      <p className={styles.noticeText}>
        {expanded ? fullText : shortText}
      </p>

      <span
        className={styles.toggleBtn}
        onClick={() => setExpanded(!expanded)}
      >
        {expanded ? "See less ▲" : "See more ▼"}
      </span>
    </div>
  );
}
