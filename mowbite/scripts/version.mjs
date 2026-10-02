// writes public/version.json before the build, so a running container can tell which version it is
// (the android app asks the mower for it). dev builds add e.g. "dev.42" via VERSION_SUFFIX
import {readFileSync, writeFileSync} from 'node:fs';

const {version} = JSON.parse(readFileSync('package.json', 'utf8'));
const suffix = process.env.VERSION_SUFFIX;
writeFileSync('public/version.json', JSON.stringify({version: suffix ? `${version}-${suffix}` : version}) + '\n');
