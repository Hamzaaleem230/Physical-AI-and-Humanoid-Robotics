import React from 'react';
// ✅ useAuth hook ko import karein (jo error handling karta hai)
import { useAuth } from '../../../auth/context/AuthProvider';

export default function AuthNavbarItem() {
  // ✅ useAuth hook se values nikaalein
  const { user, logout, isLoading } = useAuth();

  if (isLoading) {
    return <span>Loading...</span>;
  }

  return user ? (
    <button
      onClick={logout}
      style={{ background: 'transparent', border: 'none', cursor: 'pointer' }}
    >
            Logout ({user.full_name})    {' '}
    </button>
  ) : (
    // Agar aap login/signin modal use kar rahe hain, toh yahan logic add karna hoga.
    // Abhi ke liye, signin modal ka button hi rehne dete hain.
    <a href="/login" style={{ textDecoration: 'none' }}>
            Login    {' '}
    </a>
  );
}
