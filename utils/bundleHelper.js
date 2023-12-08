import $RefParser from "@apidevtools/json-schema-ref-parser";
import * as fs from "fs";

let filePath = process.argv[2];
let outputFileName = process.argv[3];

$RefParser.bundle(filePath).then(async (schema) => {

    let jsonSchema = decodeURI(JSON.stringify(schema, null, 2));
    jsonSchema = jsonSchema.replaceAll('%24', '$');

    $RefParser.dereference(JSON.parse(jsonSchema), {
      dereference: {
        circular: 'ignore',
      }}).then((derefSchema) => {

          fs.writeFile(outputFileName, JSON.stringify(derefSchema, null, 2), 'utf8', (err) => {
              if (err) throw err;
            console.log('The file has been saved!');
          });
    })

});


