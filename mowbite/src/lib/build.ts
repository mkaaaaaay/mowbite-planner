// dev builds carry a suffix (1.2.0-dev.12, 1.2.0-local), releases don't. things still being tried out
// only show up there
export const devBuild = (process.env.APP_VERSION ?? '').includes('-');
