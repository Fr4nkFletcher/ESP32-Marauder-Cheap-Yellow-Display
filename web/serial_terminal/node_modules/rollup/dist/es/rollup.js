/*
  @license
	Rollup.js v4.13.0
	Tue, 12 Mar 2024 05:27:08 GMT - commit 1c8afed74bd81cd38ad0b373ea6b6ec382975013

	https://github.com/rollup/rollup

	Released under the MIT License.
*/
export { version as VERSION, defineConfig, rollup, watch } from './shared/node-entry.js';
import './shared/parseAst.js';
import '../native.js';
import 'node:path';
import 'path';
import 'node:process';
import 'node:perf_hooks';
import 'node:fs/promises';
import 'tty';
