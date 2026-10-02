import {isApp} from './native';

// a text file for the user: a normal download in the browser, in the android app (where a webview ignores downloads)
// it goes to the app's cache and the share sheet comes up, from there it can be saved to downloads, drive, mail etc.
export async function saveFile(name: string, text: string, type = 'application/json') {
  if (isApp()) {
    const [{Filesystem, Directory, Encoding}, {Share}] = await Promise.all([
      import('@capacitor/filesystem'),
      import('@capacitor/share'),
    ]);
    const {uri} = await Filesystem.writeFile({path: name, data: text, directory: Directory.Cache, encoding: Encoding.UTF8});
    await Share.share({title: name, files: [uri], dialogTitle: name});
    return;
  }
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], {type}));
  a.download = name;
  // some browsers only follow a link that's in the page
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
