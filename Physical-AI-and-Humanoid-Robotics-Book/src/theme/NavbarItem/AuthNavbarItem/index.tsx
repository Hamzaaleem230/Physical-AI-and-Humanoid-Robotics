import React from 'react';
import { useAuth } from '../../../auth/context/AuthProvider';
import UserAvatar from '../../../auth/components/UserAvatar';
import styles from './AuthNavbarItem.module.css';

export default function AuthNavbarItem() {
  const { user, isAuthenticated, isLoading, openModal, logout } = useAuth();

  if (isLoading) {
    return <span className={styles.loading}>Loading...</span>;
  }

  // LOGGED IN
  if (isAuthenticated && user) {
    const goToProfile = () => {
      window.location.href = '/profile'; // Safe and simple
    };

    return (
      <div className={styles.container}>
        <div className={styles.userInfo} onClick={goToProfile}>
          <UserAvatar fullName={user.full_name} />
          <span className={styles.username}>{user.full_name.split(' ')[0]}</span>
        </div>

        <button className={styles.logout} onClick={logout}>
          Logout
        </button>
      </div>
    );
  }

  // LOGGED OUT
  return (
    <div className={styles.container}>
      <button className={styles.signin} onClick={() => openModal('login')}>
        Sign In
      </button>

      <button className={styles.signup} onClick={() => openModal('signup')}>
        Sign Up
      </button>
    </div>
  );
}
