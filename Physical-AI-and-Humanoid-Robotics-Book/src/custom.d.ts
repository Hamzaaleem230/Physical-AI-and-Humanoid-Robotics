// src/custom.d.ts

// Yeh declaration file TypeScript ko batata hai
// ki jab bhi woh koi file jiska naam *.module.css se end ho
// usko import kare, toh woh file asal mein
// ek object export karti hai jahan keys strings hain (yani CSS class names).

declare module '*.module.css' {
  const classes: { [key: string]: string };
  export default classes;
}