import React from 'react';
import OriginalMobileSidebar from '@theme-original/Navbar/MobileSidebar';
import { useAuth } from '../../auth/context/AuthProvider';
import UserAvatar from '../../auth/components/UserAvatar';
import styles from './MobileAuth.module.css';

console.log('✅ CUSTOM MOBILE SIDEBAR LOADED');

export default function MobileSidebar(props) {
  const { user, isAuthenticated, isLoading, openModal, logout } = useAuth();

  return (
    <>
      <OriginalMobileSidebar {...props} />

      <div className={styles.mobileAuthWrapper}>
        {isLoading && <span>Loading...</span>}

        {!isLoading && !isAuthenticated && (
          <>
            <button className={styles.signin} onClick={() => openModal('login')}>
              Sign In
            </button>

            <button className={styles.signup} onClick={() => openModal('signup')}>
              Sign Up
            </button>
          </>
        )}

        {!isLoading && isAuthenticated && user && (
          <div className={styles.userRow}>
            <div className={styles.userInfo} onClick={() => (window.location.href = '/profile')}>
              <UserAvatar fullName={user.full_name} />
              <span>{user.full_name.split(' ')[0]}</span>
            </div>

            <button className={styles.logout} onClick={logout}>
              Logout
            </button>
          </div>
        )}
      </div>
    </>
  );
}
