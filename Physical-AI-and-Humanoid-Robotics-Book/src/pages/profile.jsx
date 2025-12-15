import React from 'react';
import Layout from '@theme/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { useAuth } from '../auth/context/AuthProvider';
import styles from './Profile.module.css';

function ProfileContent() {
  const { user, isAuthenticated, isLoading } = useAuth();

  if (isLoading) {
    return <p className={styles.loading}>Loading profile...</p>;
  }

  if (!isAuthenticated || !user) {
    return <p className={styles.error}>Please log in to view your profile.</p>;
  }

  return (
    <div className={styles.profileCard}>
      <h1>Welcome, {user.full_name.split(' ')[0]}!</h1>

      {Object.entries({
        'Full Name': user.full_name,
        Email: user.email,
        'Skill Level': user.skill_level || 'N/A',
        Hardware: user.hardware || 'N/A',
        'Robotics Experience': user.robotics_experience || 'N/A',
        OS: user.os || 'N/A',
        'Learning Mode': user.learning_mode || 'N/A',
      }).map(([label, value]) => (
        <div key={label} className={styles.detailRow}>
          <span className={styles.label}>{label}:</span>
          <span className={styles.value}>{value}</span>
        </div>
      ))}
    </div>
  );
}

export default function ProfilePage() {
  return (
    <Layout title="User Profile">
      <div className={styles.profileContainer}>
        <BrowserOnly>{() => <ProfileContent />}</BrowserOnly>
      </div>
    </Layout>
  );
}
