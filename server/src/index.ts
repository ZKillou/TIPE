import express from "express";
import { writeFile } from "fs/promises";
import { PostRequestData } from "./types";

const PORT = 3035;
const app = express();

app.use(express.json());

app.post("/", async (req, res) => {
  /*
  Les valeurs du corps ne sont pas vérifiées car
  ce n'est pas le but du programme.
  */
  let body: PostRequestData = req.body;
  let date = new Date();
  await writeFile(`./res/${date.toISOString().replaceAll(":", "-")}.json`, JSON.stringify(body, null, 2), { encoding: "utf-8" });
  res.sendStatus(200);
});

app.listen(PORT);
