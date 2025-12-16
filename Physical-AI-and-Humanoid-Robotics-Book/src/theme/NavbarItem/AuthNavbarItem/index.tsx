import React, { useEffect, useState } from 'react';
import { useAuth } from '../../../auth/context/AuthProvider';
import UserAvatar from '../../../auth/components/UserAvatar';
import styles from './AuthNavbarItem.module.css';
import BrowserOnly from '@docusaurus/BrowserOnly';

function AuthNavbarContent() {
  const { user, isAuthenticated, isLoading, openModal, logout } = useAuth();
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);
  }, []);

  // 🔹 Glitch Fix: Loading state mein kuch bhi render mat karein 
  // CSS ka min-width space reserve rakhega
  if (!mounted || isLoading) {
    return <div className="authNavbarItem" style={{ visibility: 'hidden' }}></div>;
  }

  if (isAuthenticated && user) {
    return (
      <div className="authNavbarItem">
        <div 
          className={styles.userInfo} 
          onClick={() => window.location.href = '/profile'}
          style={{ cursor: 'pointer', display: 'flex', alignItems: 'center', gap: '8px' }}
        >
          <UserAvatar fullName={user.full_name} />
          <span className={styles.username} style={{ fontWeight: 600 }}>
            {user.full_name.split(' ')[0]}
          </span>
        </div>
        <button className={styles.logout} onClick={logout}>
          Logout
        </button>
      </div>
    );
  }

  return (
    <div className="authNavbarItem">
      <button className={styles.signin} onClick={() => openModal('login')}>
        Sign In
      </button>
      <button className={styles.signup} onClick={() => openModal('signup')}>
        Sign Up
      </button>
    </div>
  );
}

export default function AuthNavbarItem() {
  return (
    <BrowserOnly fallback={null}>
      {() => <AuthNavbarContent />}
    </BrowserOnly>
  );
}