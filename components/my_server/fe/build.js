import { minify } from 'html-minifier';
import fs from 'fs';

const originalHtml = fs.readFileSync('./index.dev.html', 'utf8');

const minifiedHtml = minify(originalHtml, {
  collapseWhitespace: true,
  removeComments: true,
  collapseBooleanAttributes: true,
  useShortDoctype: true,
  removeEmptyAttributes: true,
  removeOptionalTags: true,
  minifyJS: true,
  minifyCSS: true
});

fs.writeFileSync('./index.html', minifiedHtml);

