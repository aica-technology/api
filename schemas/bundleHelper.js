import $RefParser from "@apidevtools/json-schema-ref-parser";
import * as fs from "fs";

const filePath = process.argv[2];
const outputName = process.argv[3];

/**
 * Recursively delete all nested keys in an object.
 * The object must not have circular references (i.e. it should be parsed from JSON).
 * A maximum recursion depth can be specified.
 * @param obj The object to search
 * @param key The name of the key to find and delete recursively
 * @param maxDepth The maximum recursion depth
 * @param depth The initial recursion depth
 * @returns {*}
 */
const dropKeys = (obj, key, maxDepth = 10, depth = 0) => {
    if (depth >= maxDepth) {
        return;
    }
    if (typeof obj === "object" && obj !== null) {
        if (key in obj) {
            delete obj[key];
        } else {
            Object.values(obj).reduce((acc, value) => {
                if (acc !== undefined) {
                    return acc;
                }
                dropKeys(value, key, depth + 1);
            }, undefined);
        }
    }
    return obj;
};

const saveFile = (filename, content) =>
    fs.writeFile(filename, content, "utf8", (err) => {
        if (err) {
            throw err;
        }
        console.log(`The file has been saved as ${filename}`);
    });

$RefParser.bundle(filePath).then(async (schema) => {
    let jsonSchema = decodeURI(JSON.stringify(schema, null, 2));
    jsonSchema = jsonSchema.replaceAll("%24", "$");
    saveFile(`${outputName}.schema.json`, jsonSchema);

    // generate a minimal version with less validation to simplify type generation, particularly for Go types
    // -> by re-parsing from JSON we can also guarantee no circular references
    const schemaObject = JSON.parse(jsonSchema);

    // remove any allOf statements that can cause ambiguity in Go type generation
    dropKeys(schemaObject, "allOf");

    // for the extension schema, patch the payload schema to be more permissive
    if (filePath.endsWith("extension.schema.json")) {
        delete schemaObject.$defs?.service?.properties?.payload_schema?.properties;
        delete schemaObject.$defs?.service?.properties?.payload_schema?.required;
    }

    jsonSchema = JSON.stringify(schemaObject, null, 2);
    saveFile(`${outputName}.types.schema.json`, jsonSchema);
});
