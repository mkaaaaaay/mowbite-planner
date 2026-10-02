// kept apart from theme.ts, the layout (a server component) needs this without react hooks
export const THEME_KEY = 'theme';

// inline in <head>, so the theme is there before the first paint: dark unless light, frost or auto was picked
export const THEME_BOOT_SCRIPT = `var t='dark';try{t=localStorage.getItem('${THEME_KEY}')||t}catch(e){}if(t!=='auto')document.documentElement.dataset.theme=t==='light'||t==='frost'?t:'dark'`;
