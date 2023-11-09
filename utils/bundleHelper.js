import $RefParser from "@apidevtools/json-schema-ref-parser";

export async function bundleHelper(filePath) {
    let bundling = await $RefParser.bundle(filePath).then((schema) => {
        let jsonSchema = JSON.stringify(schema, null, 2);
        jsonSchema = jsonSchema.replaceAll('%24', '$');
        return jsonSchema
    });
    return bundling
}