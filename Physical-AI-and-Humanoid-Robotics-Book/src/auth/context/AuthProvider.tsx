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
  // 🔹 SAFE INITIALIZATION: Window check zaroori hai build error se bachne ke liye
  const [user, setUser] = useState<AuthUserProfile | null>(() => {
    if (typeof window !== 'undefined') {
      const saved = localStorage.getItem('authUser');
      return saved ? JSON.parse(saved) : null;
    }
    return null;
  });

  // State initialization
  const [isLoading, setIsLoading] = useState(true); 
  const isAuthenticated = !!user;

  // Modal State
  const [isSigninOpen, setIsSigninOpen] = useState(false);
  const [isSignupOpen, setIsSignupOpen] = useState(false);

  // 🔹 SAFE LOCALSTORAGE SYNC: Browser check ke sath
  useEffect(() => {
    if (typeof window !== 'undefined') {
      if (user) {
        localStorage.setItem('authUser', JSON.stringify(user));
      } else {
        localStorage.removeItem('authUser');
      }
    }
  }, [user]);

  // 🔹 LOAD USER FROM API: Build safe version
  useEffect(() => {
    const loadUser = async () => {
      // Agar user pehle se hai (LocalStorage se), to seedha loading false kar do
      if (user) {
        setIsLoading(false);
        return;
      }

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
  }, []); // Run only once on mount

  // 🔹 LOGIN FUNCTION (As It Is)
  const login = async (email: string, password: string) => {
    setIsLoading(true);
    try {
      await apiSignin({ email, password });
      const freshUser = await getCurrentUser();
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

  // 🔹 LOGOUT FUNCTION (As It Is)
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

  // 🔹 MODAL FUNCTIONS (As It Is)
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