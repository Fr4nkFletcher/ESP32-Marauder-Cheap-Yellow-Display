/**
 * @name slipEncode
 * Take an array buffer and return back a new array where
 * 0xdb is replaced with 0xdb 0xdd and 0xc0 is replaced with 0xdb 0xdc
 */
export declare const slipEncode: (buffer: number[]) => number[];
/**
 * @name toByteArray
 * Convert a string to a byte array
 */
export declare const toByteArray: (str: string) => number[];
export declare const hexFormatter: (bytes: number[]) => string;
export declare const toHex: (value: number, size?: number) => string;
export declare const sleep: (ms: number) => Promise<unknown>;
