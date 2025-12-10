import React from 'react';
import styles from './UserAvatar.module.css';

interface UserAvatarProps {
  fullName: string;
}

const UserAvatar: React.FC<UserAvatarProps> = ({ fullName }) => {
  const initial = fullName ? fullName.charAt(0).toUpperCase() : '?';

  return (
    <div
      className={styles.avatarContainer} 
      title={fullName}
    >
      {initial}
    </div>
  );
};

export default UserAvatar;