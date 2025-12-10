import React from 'react';
import { useAuth } from '../context/AuthProvider';
import UserAvatar from './UserAvatar';
import styles from './AuthStatusDisplay.module.css'; 

const AuthStatusDisplay: React.FC = () => {
  const { user, logout, isAuthenticated, isLoading, openModal } = useAuth();

  if (isLoading) return <span className={styles.loadingText}>Loading...</span>; 

  return isAuthenticated && user ? (
    // LOGGED IN STATE
    <div className={styles.authContainer}> 
      
      {/* Profile Link Button */}
      <button
        onClick={() => window.location.href = "/profile"} 
        className={styles.authButton + ' ' + styles.profileLink} 
        aria-label="View Profile"
      >
        <UserAvatar fullName={user.full_name} />
        {user.full_name.split(' ')[0]}
      </button>

      {/* Logout Button */}
      <button
        onClick={logout}
        className={styles.authButton + ' ' + styles.logoutButton} 
      >
        Logout
      </button>
    </div>
  ) : (
    // LOGGED OUT STATE
    <button
      className={styles.authButton}
      onClick={() => openModal('login')}
    >
      Login / Sign Up
    </button>
  );
};

export default AuthStatusDisplay;