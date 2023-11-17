import $RefParser from "@apidevtools/json-schema-ref-parser";
import * as fs from "fs";

let filePath = process.argv[2];
let outputFileName = process.argv[3];
$RefParser.bundle(filePath).then(async (schema) => {
    let jsonSchema = JSON.stringify(schema, null, 2);
    jsonSchema = jsonSchema.replaceAll('%24', '$');
    jsonSchema = jsonSchema.replaceAll('(%5E%5Ba-zA-Z%5D%5Ba-zA-Z0-9_%5D*%5Ba-zA-Z0-9%5D$)%7C(%5E%5Ba-zA-Z%5D$)', '(^[a-zA-Z][a-zA-Z0-9_]*[a-zA-Z0-9]$)|(^[a-zA-Z]$)');

    fs.writeFile(outputFileName, jsonSchema, 'utf8', (err) => {
        if (err) throw err;
        console.log('The file has been saved!');
    });
});


