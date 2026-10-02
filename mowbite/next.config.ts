import type { NextConfig } from "next";
import { readFileSync } from "node:fs";

// the version from package.json, dev builds add e.g. "dev.42" (VERSION_SUFFIX, set by the github actions)
const version = JSON.parse(readFileSync("./package.json", "utf8")).version as string;
const suffix = process.env.VERSION_SUFFIX;

const nextConfig: NextConfig = {
  output: "export",
  trailingSlash: true,
  // lets phones etc. on the lan use the dev server, e.g. DEV_ORIGINS=myhost,192.168.1.20
  allowedDevOrigins: process.env.DEV_ORIGINS?.split(",") ?? [],
  env: {
    APP_VERSION: suffix ? `${version}-${suffix}` : version,
  },
};

export default nextConfig;
