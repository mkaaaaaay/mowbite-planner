// copies in plain http too, where navigator.clipboard isn't there
export function copyText(text: string) {
  if (navigator.clipboard) return void navigator.clipboard.writeText(text).catch(() => {});
  const t = document.createElement('textarea');
  t.value = text;
  document.body.appendChild(t);
  t.select();
  document.execCommand('copy');
  t.remove();
}
