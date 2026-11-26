"""Build the NGCC ASIC Leaderboard static site.

usage: python3 build.py [output_dir]          (default: dist/)

Every top-level directory with a category.json is one category page. Each
<category>/<implementation>/result.json holds the latest result of one
implementation. The data are validated first; the build fails on any error.
Output: <output_dir>/<category>/index.html and data.json, plus a root index.html
that redirects to the first category with results.
"""
import base64
import datetime
import html
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent
OUT = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'dist'
REPO_URL = 'https://github.com/chipweaver/ngcc-asic-leaderboard'

# NGCC categories in the order of the official documents (公钥密码算法提交要求: 数字签名、密钥封装、密钥交换; then 密码杂凑)
CATEGORIES = [
    ('sign', {'zh': '数字签名', 'en': 'Digital Signature'}),
    ('kem', {'zh': '密钥封装', 'en': 'Key Encapsulation'}),
    ('kex', {'zh': '密钥交换', 'en': 'Key Exchange'}),
    ('hash', {'zh': '密码杂凑', 'en': 'Cryptographic Hash'}),
]
VERSIONS = {'performance'}
FONTS = [('Fragment Mono', 400, 'fragment-mono-latin-400-normal.woff2'),
         ('DM Sans', 400, 'dm-sans-latin-400-normal.woff2'),
         ('DM Sans', 500, 'dm-sans-latin-500-normal.woff2'),
         ('DM Sans', 600, 'dm-sans-latin-600-normal.woff2')]

errors = []


def check(cond, where, msg):
    if not cond:
        errors.append(f'{where}: {msg}')
    return cond


def load_category(cdir):
    cfg = json.loads((cdir / 'category.json').read_text(encoding='utf-8'))
    op_ids = [o['id'] for o in cfg['ops']]
    check(set(cfg['operation']) <= set(op_ids), cdir.name, 'operation lists unknown op ids')
    for lang in ('zh', 'en'):
        check(all(k in cfg['text'].get(lang, {}) for k in ('doc', 'title', 'tagline', 'method')), cdir.name, f'text.{lang} incomplete')
    results = []
    for rf in sorted(cdir.glob('*/result.json')):
        r = json.loads(rf.read_text(encoding='utf-8'))
        where = str(rf.relative_to(ROOT))
        check(r.get('version') in VERSIONS, where, f"unknown version {r.get('version')!r}")
        check(all(k in r.get('conditions', {}) for k in ('process', 'sram_model')), where, 'conditions incomplete')
        check(isinstance(r.get('frequency_mhz'), (int, float)) and r['frequency_mhz'] > 0, where, 'frequency_mhz missing')
        check(all(isinstance(r.get('area_kge', {}).get(k), (int, float)) for k in ('logic', 'sram', 'total')), where, 'area_kge incomplete')
        check((rf.parent / 'rtl').is_dir(), where, 'no rtl/ directory next to result.json')
        for p in r.get('parameter_sets', []):
            pw = f"{where} [{p.get('name')}]"
            check(isinstance(p.get('level'), int), pw, 'level missing')
            for op in op_ids:
                c = p.get('cycles', {}).get(op)
                if check(c is not None, pw, f'cycles.{op} missing'):
                    check(c.get('mean', 0) > 0 and c.get('n', 0) > 0, pw, f'cycles.{op} needs mean > 0 and n > 0')
                    if cfg.get('show_p95'):
                        check(isinstance(c.get('p95'), (int, float)), pw, f'cycles.{op}.p95 missing')
            for s in cfg['sizes']:
                check(s['id'] in p.get('sizes_bytes', {}), pw, f"sizes_bytes.{s['id']} missing")
        r['slug'] = rf.parent.name
        results.append(r)
    # the evaluation conditions are common to all results; the clock frequency is each implementation's own
    conds = {json.dumps(r['conditions'], sort_keys=True) for r in results}
    check(len(conds) <= 1, cdir.name, 'results use different evaluation conditions')
    return cfg, results


def board(cfg, results):
    """One row per (level, algorithm): the parameter set with the highest throughput."""
    rows = []
    for r in results:
        kge, f_mhz = r['area_kge']['total'], r['frequency_mhz']
        for p in r['parameter_sets']:
            if p['level'] not in cfg['levels']:
                continue                       # declared at a level that is not an NGCC level: kept in data, not ranked
            name = p['name']
            short = name[len(r['algorithm']) + 1:] if name.startswith((r['algorithm'] + '-', r['algorithm'] + '_')) else name
            tput = f_mhz * 1e6 / sum(p['cycles'][op]['mean'] for op in cfg['operation'])
            rows.append(dict(
                alg=r['algorithm'], slug=r['slug'], param=name, level=p['level'],
                short='' if short == str(p['level']) else short,
                freq=f_mhz, tput=tput, kge=kge, tpa=tput / kge,
                eff=None,    # 能效比 ops/s/mW: left empty until power is measured under the guide's conditions
                cyc={o['id']: p['cycles'][o['id']]['mean'] for o in cfg['ops']},
                p95={o['id']: p['cycles'][o['id']].get('p95') for o in cfg['ops']},
                n={o['id']: p['cycles'][o['id']]['n'] for o in cfg['ops']},
                area=r['area_kge'], sizes=p['sizes_bytes'], power=p.get('power_mw') or {},
            ))
    best = {}
    for x in rows:
        k = (x['level'], x['alg'])
        if k not in best or x['tput'] > best[k]['tput']:
            best[k] = x
    for x in rows:
        b = best[(x['level'], x['alg'])]
        if x is not b:
            b.setdefault('alts', []).append(x['param'])
    for b in best.values():
        b['alts'] = sorted(b.get('alts', []))
    levels = sorted({lv for lv, _ in best})
    return {str(lv): sorted((b for (l2, _), b in best.items() if l2 == lv), key=lambda b: -b['tput']) for lv in levels}


def main():
    live = {}
    for cid, _ in CATEGORIES:
        cdir = ROOT / cid
        if (cdir / 'category.json').is_file():
            cfg, results = load_category(cdir)
            if results:
                live[cid] = (cfg, results)
    for d in ROOT.iterdir():
        if d.is_dir() and (d / 'category.json').is_file():
            check(d.name in dict(CATEGORIES), d.name, 'not a known NGCC category id')
    check(bool(live), 'site', 'no category has results')
    if errors:
        sys.exit('data check failed:\n  ' + '\n  '.join(errors))

    font_css = '\n'.join(
        f"@font-face{{font-family:'{fam}';font-weight:{w};font-style:normal;font-display:swap;"
        f"src:url(data:font/woff2;base64,{base64.b64encode((ROOT / 'site' / 'fonts' / f).read_bytes()).decode()}) format('woff2')}}"
        for fam, w, f in FONTS)
    template = (ROOT / 'site' / 'template.html').read_text(encoding='utf-8')
    today = datetime.date.today().isoformat()
    nav = [[cid, cid in live, names] for cid, names in CATEGORIES]

    for cid, (cfg, results) in live.items():
        cond = results[0]['conditions']
        payload = dict(
            cat=cid, cats=nav, repo=REPO_URL, config=cfg,
            meta=dict(process=cond['process'], sram=cond['sram_model'],
                      date=today, algorithms=len({r['algorithm'] for r in results})),
            levels=board(cfg, results),
        )
        page = template.replace('/*FONTS*/', font_css).replace('/*TITLE*/', html.escape(cfg['text']['zh']['doc']))
        page = page.replace('/*DATA*/null', json.dumps(payload, ensure_ascii=False).replace('</', '<\\/'))
        out = OUT / cid
        out.mkdir(parents=True, exist_ok=True)
        (out / 'index.html').write_text(page, encoding='utf-8')
        public = dict(category=cid, generated=today,
                      results=[{k: v for k, v in r.items() if k != 'slug'} | {'id': r['slug']} for r in results])
        (out / 'data.json').write_text(json.dumps(public, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
        print(f'{out}/index.html: {sum(len(v) for v in payload["levels"].values())} rows, levels {list(payload["levels"])}')

    first = next(cid for cid, _ in CATEGORIES if cid in live)
    (OUT / 'index.html').write_text(
        f'<!doctype html><meta charset="utf-8"><title>NGCC ASIC Leaderboard</title>'
        f'<meta http-equiv="refresh" content="0; url={first}/"><a href="{first}/">{first}/</a>\n', encoding='utf-8')


if __name__ == '__main__':
    main()
