import $RefParser from "@apidevtools/json-schema-ref-parser";
import * as fs from "fs";

let filePath = process.argv[2];
let outputFileName = process.argv[3];

$RefParser.bundle(filePath).then(async (schema) => {

    let jsonSchema = decodeURI(JSON.stringify(schema, null, 2));
    jsonSchema = jsonSchema.replaceAll('%24', '$');

    if (outputFileName === "application.schema.json") {
        fs.writeFile(outputFileName, jsonSchema, 'utf8', (err) => {
            if (err) throw err;
            console.log('The file has been saved!');
        });
    } else {
        let jsonSchemaObject = JSON.parse(jsonSchema);
        // Avoid nested reference (cannot be resolved by VSCode)
        if (jsonSchemaObject?.['properties']?.['parameters']?.['items']?.['properties']?.['parameter_state_type']?.['$ref']) {
            jsonSchemaObject['properties']['parameters']['items']['properties']['parameter_state_type']['$ref'] = '#/properties/inputs/items/properties/signal_types/items';
        }
        fs.writeFile(outputFileName, JSON.stringify(jsonSchemaObject, null, 2), 'utf8', (err) => {
            if (err) throw err;
            console.log('The file has been saved!');
        });
    }
});


