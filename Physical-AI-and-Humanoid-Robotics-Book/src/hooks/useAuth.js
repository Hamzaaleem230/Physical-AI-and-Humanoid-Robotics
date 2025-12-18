import { useEffect, useState } from 'react';

export default function useAuth() {
  const [jwtToken, setJwtToken] = useState(null);
  const [userId, setUserId] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    try {
      // Real users ka data localStorage se aayega
      const token = localStorage.getItem('jwtToken');
      const storedUserId = localStorage.getItem('userId');

      if (token && storedUserId) {
        setJwtToken(token);
        setUserId(storedUserId);
      } else {
        // Sirf testing ke liye hf-user-001 rakhein, production mein isay null kar dein
        setUserId(null); 
      }
    } catch (error) {
      console.error('useAuth error:', error);
    } finally {
      setLoading(false);
    }
  }, []);

  return {
    jwtToken,
    userId,
    isAuthenticated: Boolean(jwtToken),
    loading,
  };
}