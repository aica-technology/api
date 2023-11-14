import $RefParser from "@apidevtools/json-schema-ref-parser";
import * as fs from "fs";

let filePath = process.argv[2];
let outputFileName = process.argv[3];
$RefParser.bundle(filePath).then((schema) => {
    let jsonSchema = JSON.stringify(schema, null, 2);
    jsonSchema = jsonSchema.replaceAll('%24', '$');
    fs.writeFile(outputFileName, jsonSchema, 'utf8', (err) => {
      if (err) throw err;
      console.log('The file has been saved!');
    });
});



