# CTPS cargo layouts

`layout.json` files authored by the offline **layout editor** live here — the
dedicated home for cargo layouts (field objects + robot manipulator zones), kept
separate from the Python pipeline and from the generated players in `../output/`.

## Flow

1. **Edit** — `python -m tools.path_planner.ctps --editor` (VSCode task
   *"CTPS: Edit Layout (open editor)"*) opens the editor. Author objects on the
   Field tab and zones on the Robot tab.
2. **Save** — the editor's Save downloads a `layout.json`. Point the browser's
   Save dialog at **this folder** once; leave *"ask where to save each file"* ON
   so your global download location is untouched — the dialog remembers this
   folder and defaults here on every later Save.
3. **View** — `python -m tools.path_planner.ctps` (VSCode task *"CTPS: build &
   view"*) auto-embeds the **newest** `layout*.json` from this folder into the
   player, so it auto-loads on open — no in-browser "Load layout…" upload.
   Override with `--layout <file>`.

## Notes

- Chrome suffixes repeat saves of the same name as `layout (1).json`,
  `layout (2).json`, … The **newest** file wins, so you never rename or delete
  the older ones.
- The working `layout*.json` files are gitignored (per-machine authoring state).
  `git add -f layouts/layout.json` if you ever want to commit a canonical
  competition layout.
