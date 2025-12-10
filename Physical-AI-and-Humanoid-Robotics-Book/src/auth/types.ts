export interface UserCreate {
  email: string;
  password: string;
  full_name: string;
  skill_level?: string;
  hardware?: string;
  robotics_experience?: string;
  os?: string;
  learning_mode?: string;
}

export interface UserLogin {
  email: string;
  password: string;
}

export interface AuthUserProfile {
  email: string;
  full_name: string;
  skill_level?: string;
  hardware?: string;
  robotics_experience?: string;
  os?: string;
  learning_mode?: string;
}
