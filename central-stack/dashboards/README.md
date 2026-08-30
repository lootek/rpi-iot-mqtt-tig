# Dashboards: nightly backup to git, manual restore

The six dashboards existed only in `grafana.db` on lothlorien — an SD card in a
Pi with 305 days uptime and no backup. This exports them to git nightly so they
are versioned and recoverable, while editing continues to happen in the UI.

```sh
./export.py                 # pull Grafana -> files, report changes
./export.py --commit        # ...and commit + push if anything changed
./restore.py --list         # compare git against live
./restore.py --all          # push files -> Grafana (recovery)
```

## The direction is deliberately opposite to alerting/

| | source of truth | direction | trigger |
|---|---|---|---|
| `central-stack/alerting/` | **git** | `apply.py` pushes git → Grafana | manual |
| `central-stack/dashboards/` | **Grafana** | `export.py` pulls Grafana → git | nightly cron |

Dashboards are edited in the UI, so git is the *backup and history*, not the live
source. Restoring is a deliberate act, never automatic.

### Why not file provisioning

It looks like the obvious answer and it is the wrong one here. Grafana's docs:
with `allowUiUpdates: true` you can save from the UI, but *"if a provisioned
dashboard is saved from the UI and then later updated from the source, the
dashboard stored in the database will always be overwritten."* Against
`Weather conditions` at v56, that is a work-loss machine. Dashboards stay
ordinary DB dashboards.

## Layout

```
central-stack/dashboards/
├── export.py  restore.py  lib.py
├── folders.json                    # folder title -> uid, so a restore rebuilds folders
├── Host monitoring/
│   ├── 000000127-nodes.json
│   ├── alerts-overview-alerts-overview.json
│   └── b3404a4a-…-docker.json
└── MQTT/  (room / weather / soil / electrical)
```

On-disk folders mirror Grafana's, so diffs read naturally and a restore onto a
fresh Grafana recreates the folder structure instead of dumping everything into
General.

## Determinism matters more than it looks

The nightly cron only commits when something actually changed, which requires an
unedited dashboard to export byte-identically every time. Two things secure that:

- `json.dumps(sort_keys=True, indent=2)` — the API's map ordering is not stable
  across calls, and without sorting cron would commit churn every night.
- **`id` is stripped.** It is a per-instance autoincrement primary key; it differs
  between any two Grafana installs and would make a restored dashboard produce a
  spurious diff forever after.

`uid` and `version` are deliberately **kept**. `uid` is what makes a restore land
on top of the existing dashboard, preserving alert annotations and saved links.
`version` increments on every UI save, so it is exactly the signal that says
"someone changed this".

Verify with: run `./export.py` twice, confirm `git status` is clean the second time.

## The nightly job runs in an isolated clone

`~/rpi-iot-mqtt-tig` on lothlorien is the **live deployment** — the compose files,
telegraf config and volumes actually running the IoT stack. A `git pull --rebase`
there would drag unreviewed upstream changes into the running system and collide
with its untracked working files. So the cron never goes near it:

```
/home/pi/dashboard-sync/repo     <- sparse clone, central-stack/dashboards only
```

Three layers keep that true:

1. **Sparse, blobless clone** — the working tree contains only
   `central-stack/dashboards/`, so it cannot function as a deployment.
2. **Hard guard** — `lib.assert_not_live_tree` refuses to run git operations if
   `--repo` resolves to a known live deployment path, so it cannot be misaimed.
   It also warns when run from a full checkout.
3. **Token outside every repo tree**, at
   `/home/pi/.config/grafana-dashboard-sync/env` (mode 600), so an unattended
   commit to a public repo cannot reach it even if a `.gitignore` broke.

Cron entry:

```
30 3 * * * /usr/bin/env python3 /home/pi/dashboard-sync/repo/central-stack/dashboards/export.py --commit --pull --repo /home/pi/dashboard-sync/repo >> /home/pi/.cache/dashboard-export.log 2>&1
```

## Secret pre-flight

The repo is public and the nightly push is unattended, so the staged diff is
scanned before committing and the commit aborts (unstaging everything) on a hit.
Dashboards carry only datasource *uids* today, but nothing stops a future panel
embedding a credential in a URL.

Covers Grafana/GitHub tokens, AWS keys, private keys, `user:pass@host` URLs, and
`"password": "…"` style assignments. The JSON-quoted form is the one that matters
here and was the case a first pass missed — dashboards are JSON, so a leak looks
like `"password": "value"`, not `password=value`.

Tested to produce zero false positives across all exported dashboards; a
trigger-happy guard would block every nightly commit and train you to ignore it.

## Recovery

```sh
./restore.py --list                                   # what differs
./restore.py --all --dry-run                          # show, change nothing
./restore.py --all                                    # full recovery
./restore.py --uid aem53k0pia29sd                     # one dashboard
./restore.py --uid X --as-uid scratch --folder General # safe test copy
```

`--as-uid` exists so restore can be exercised without touching a real dashboard:
it lands a copy under a throwaway uid which you then delete. Use it rather than
test-restoring over something you care about.

## Alerts overview

`alerts-overview` (Host monitoring folder) is a fleet-wide view built only on the
built-in `-- Grafana --` datasource, so it has no InfluxDB dependency and cannot
break when a datasource uid changes.

| panel | shows |
|---|---|
| Firing now | firing + error, sorted by importance |
| Pending / no data | what is about to fire |
| All rules, grouped by severity | every rule including Normal, as stat tiles |
| Recent alert state changes | alert history over the dashboard time range |

The history panel is why this beats a plain alert list: `repeat_interval` is 12h
and `severity=info` is muted 23:00–07:00, so an alert can fire and resolve without
ever reaching the inbox. This is where that becomes visible.

It is exported like any other dashboard — edit it in the UI, and the nightly job
picks the change up.
