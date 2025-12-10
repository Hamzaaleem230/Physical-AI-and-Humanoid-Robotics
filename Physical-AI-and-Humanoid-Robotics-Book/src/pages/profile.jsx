import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../auth/context/AuthProvider';
import styles from './Profile.module.css';

function ProfilePage() {
  const { user, isAuthenticated, isLoading } = useAuth();
  

  const [profileData, setProfileData] = useState(user); 

  if (isLoading) {
    return (
      <Layout title="Profile">
        <div className={styles.profileContainer}>
          <p className={styles.loading}>Loading profile...</p>
        </div>
      </Layout>
    );
  }

  if (!isAuthenticated || !user) {
    return (
      <Layout title="Profile">
        <div className={styles.profileContainer}>
          <p className={styles.error}>Please log in to view your profile.</p>
        </div>
      </Layout>
    );
  }

  // All fields are available for display
  const displayData = {
    'Full Name': user.full_name,
    'Email': user.email,
    'Skill Level': user.skill_level || 'N/A',
    'Hardware': user.hardware || 'N/A',
    'Robotics Experience': user.robotics_experience || 'N/A',
    'OS': user.os || 'N/A',
    'Learning Mode': user.learning_mode || 'N/A',
  };

  return (
    <Layout title="User Profile" description="View and manage your user profile details.">
      <div className={styles.profileContainer}>
        <div className={styles.profileCard}>
          <h1>Welcome, {user.full_name.split(' ')[0]}!</h1>
          
          {Object.entries(displayData).map(([label, value]) => (
            <div key={label} className={styles.detailRow}>
              <span className={styles.label}>{label}:</span>
              <span className={styles.value}>{value}</span>
            </div>
          ))}
          
        </div>
      </div>
    </Layout>
  );
}

export default ProfilePage;