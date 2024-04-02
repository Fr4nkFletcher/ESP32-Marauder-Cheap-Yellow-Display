import { nodeResolve } from "@rollup/plugin-node-resolve";
import json from "@rollup/plugin-json";
import terser from "@rollup/plugin-terser";

const config = {
  input: "dist/index.js",
  output: {
    dir: "dist/web",
    format: "module",
  },
  // preserveEntrySignatures: false,
  plugins: [nodeResolve(), json()],
};

if (process.env.NODE_ENV === "production") {
  config.plugins.push(
    terser({
      ecma: 2019,
      toplevel: true,
      output: {
        comments: false,
      },
    })
  );
}

export default config;
