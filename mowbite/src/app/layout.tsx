import AppSetup from "@/components/AppSetup";
import Grass from "@/components/Grass";
import Nav from "@/components/Nav";
import SwipeNav from "@/components/SwipeNav";
import EdgeBounce from "@/components/EdgeBounce";
import FreshPage from "@/components/FreshPage";
import { COLOR_BOOT_SCRIPT } from "@/lib/settings";
import { THEME_BOOT_SCRIPT } from "@/lib/themeBoot";
import type { Metadata, Viewport } from "next";
import { Geist, Geist_Mono } from "next/font/google";
import "./globals.css";
import styles from "./layout.module.css";

const geistSans = Geist({
  variable: "--font-geist-sans",
  subsets: ["latin"],
});

const geistMono = Geist_Mono({
  variable: "--font-geist-mono",
  subsets: ["latin"],
});

// cover: the tab bar reaches under the phone's home indicator, env(safe-area-inset-bottom) keeps it clear
export const viewport: Viewport = {
  width: "device-width",
  initialScale: 1,
  viewportFit: "cover",
  themeColor: "#101114",
};

export const metadata: Metadata = {
  // name under the icon when the app is put on the home screen (iOS reads this, android the manifest)
  appleWebApp: { title: "MowBite", capable: true, statusBarStyle: "black" },
  title: "MowBite",
  description: "A web app for OpenMower robot mowers",
};

export default function RootLayout({ children }: LayoutProps<"/">) {
  return (
    // the color script in <head> sets css variables on <html> before hydration
    <html lang="en" className={`${geistSans.variable} ${geistMono.variable}`} suppressHydrationWarning>
      <head>
        {/* eslint-disable-next-line @next/next/no-sync-scripts -- has to run before the app connects */}
        <script src="/config.js" />
        <script dangerouslySetInnerHTML={{ __html: COLOR_BOOT_SCRIPT + THEME_BOOT_SCRIPT }} />
      </head>
      <body>
        <div className={styles.shell}>
          <Nav />
          <div className={styles.content} data-page>
            {children}
          </div>
          <Grass />
          <AppSetup />
          <SwipeNav />
          <EdgeBounce />
          <FreshPage />
        </div>
      </body>
    </html>
  );
}
