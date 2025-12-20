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
  }, []); 

  // 🔹 LOGIN FUNCTION (FINAL FIXED & ERROR FREE)
  const login = async (email: string, password: string) => {
    setIsLoading(true);
    try {
      // 1. Signin call se response lein (any use kiya taake TS error na de)
      const response: any = await apiSignin({ email, password });
      
      console.log("Login Response Debug:", response);

      // 2. Response se asli user_id nikal kar save karein
      if (response) {
        // Auth backend ya to user_id bhej raha hai ya id
        const finalId = response.user_id || response.id;
        
        if (finalId) {
          const idAsString = finalId.toString();
          // Dono keys update karein taake Personalization backend ko sahi ID mile
          localStorage.setItem('jwtToken', idAsString);
          localStorage.setItem('userId', idAsString);
          console.log("Storage Updated: jwtToken & userId set to", idAsString);
        }
      }

      // 3. Current user details fetch karein
      const freshUser = await getCurrentUser();
      setUser(freshUser);
      
      // 4. Modal band karein
      closeModal();
      
      // 5. Chota sa delay taake storage confirm ho jaye phir refresh
      setTimeout(() => {
        window.location.reload();
      }, 150);

    } catch (error) {
      console.error('Login failed:', error);
      setUser(null);
      throw error;
    } finally {
      setIsLoading(false);
    }
  };

  // 🔹 LOGOUT FUNCTION
  const logout = async () => {
    setIsLoading(true);
    try {
      await apiSignout();
      // Token aur ID saaf karein
      localStorage.removeItem('jwtToken');
      localStorage.removeItem('userId');
      setUser(null);
      window.location.reload();
    } catch (error) {
      console.error('Logout failed:', error);
      throw error;
    } finally {
      setIsLoading(false);
    }
  };

  // 🔹 MODAL FUNCTIONS
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
      <SigninModal isOpen={isSigninOpen} onClose={closeModal} />
      <SignupModal isOpen={isSignupOpen} onClose={closeModal} />
    </AuthContext.Provider>
  );
};

export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};