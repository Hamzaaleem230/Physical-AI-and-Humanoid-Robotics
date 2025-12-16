import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import {
  getCurrentUser,
  signout as apiSignout,
  signin as apiSignin,
} from '../services/api';
import { AuthUserProfile } from '../types';

// Import Modals for rendering
import SigninModal from '../components/SigninModal';
import SignupModal from '../components/SignupModal';

// Shape of our authentication context
export interface AuthContextType {
  user: AuthUserProfile | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (email: string, password: string) => Promise<void>;
  logout: () => Promise<void>;
  openModal: (type: 'login' | 'signup') => void;
  closeModal: () => void;
}

export const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  // 🔹 Initialize from LocalStorage for stable initial render
  const [user, setUser] = useState<AuthUserProfile | null>(() => {
    const saved = localStorage.getItem('authUser');
    return saved ? JSON.parse(saved) : null;
  });

  const [isLoading, setIsLoading] = useState(!user); // if no user in LS, start loading
  const isAuthenticated = !!user;

  // Modal State
  const [isSigninOpen, setIsSigninOpen] = useState(false);
  const [isSignupOpen, setIsSignupOpen] = useState(false);

  // 🔹 Keep LocalStorage in sync whenever user changes
  useEffect(() => {
    if (user) {
      localStorage.setItem('authUser', JSON.stringify(user));
    } else {
      localStorage.removeItem('authUser');
    }
  }, [user]);

  // 🔹 Load user from API if not in LocalStorage
  useEffect(() => {
    if (!user) {
      const loadUser = async () => {
        try {
          const currentUser = await getCurrentUser();
          setUser(currentUser);
        } catch (error) {
          console.error('Failed to load user:', error);
          setUser(null);
        } finally {
          setIsLoading(false);
        }
      };
      loadUser();
    } else {
      setIsLoading(false);
    }
  }, [user]);

  const login = async (email: string, password: string) => {
    setIsLoading(true);
    try {
      await apiSignin({ email, password });

      const freshUser = await getCurrentUser();  // always fetch fresh user
      setUser(freshUser);

      closeModal();
    } catch (error) {
      console.error('Login failed:', error);
      setUser(null);
      throw error;
    } finally {
      setIsLoading(false);
    }
  };

  const logout = async () => {
    setIsLoading(true);
    try {
      await apiSignout();
      setUser(null);
    } catch (error) {
      console.error('Logout failed:', error);
      throw error;
    } finally {
      setIsLoading(false);
    }
  };

  const openModal = (type: 'login' | 'signup') => {
    closeModal();
    if (type === 'login') setIsSigninOpen(true);
    if (type === 'signup') setIsSignupOpen(true);
  };

  const closeModal = () => {
    setIsSigninOpen(false);
    setIsSignupOpen(false);
  };

  const contextValue: AuthContextType = {
    user,
    isAuthenticated,
    isLoading,
    login,
    logout,
    openModal,
    closeModal,
  };

  return (
    <AuthContext.Provider value={contextValue}>
      {children}

      {/* AUTH MODALS */}
      <SigninModal isOpen={isSigninOpen} onClose={closeModal} />
      <SignupModal isOpen={isSignupOpen} onClose={closeModal} />
    </AuthContext.Provider>
  );
};

// Custom hook for consuming the context
export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
