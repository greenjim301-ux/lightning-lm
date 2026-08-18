import { useEffect, useRef, useState } from 'react'
import type { CreateLightningUiModule, LightningUiModule } from './lightning-ui-module'

const DEFAULT_WS_URL = 'ws://localhost:9877'
const WS_URL_STORAGE_KEY = 'lightning-ws-url'

// Vite's dev server rewrites every import() call it sees at transform time to append a
// `?import` tracking query — even with a /* @vite-ignore */ comment and a non-literal
// specifier — and then refuses that request because public/ assets (this wasm build output,
// copied in by scripts/sync-wasm.sh) aren't allowed to be `import()`-ed as source modules.
// Going through `new Function` hides the `import()` call from Vite's static transform
// entirely, so the browser performs a truly native dynamic import with no query appended.
const nativeDynamicImport = new Function('specifier', 'return import(specifier)') as (
  specifier: string,
) => Promise<{ default: CreateLightningUiModule }>

function getInitialWsUrl(): string {
  const fromQuery = new URLSearchParams(window.location.search).get('ws')
  return fromQuery || localStorage.getItem(WS_URL_STORAGE_KEY) || DEFAULT_WS_URL
}

type Status = 'loading' | 'running' | 'error'

export function LightningUiViewer() {
  const [wsUrl] = useState(getInitialWsUrl)
  const [wsUrlInput, setWsUrlInput] = useState(wsUrl)
  const [status, setStatus] = useState<Status>('loading')
  const [error, setError] = useState<string | null>(null)
  // Defaults to on, unlike the native desktop app's menu.Follow (which defaults off there
  // because its on-canvas panel makes toggling it a click away). The panel doesn't actually
  // render under this Emscripten/ES3 Pangolin build (see the embind bindings in
  // wasm_ui_client.cc this calls into), so defaulting off here would leave a first-time
  // viewer staring at a static patch of ground the vehicle drives out of within seconds —
  // easy to mistake for a broken/frozen page rather than a camera that isn't tracking yet.
  const [follow, setFollowState] = useState(true)
  const moduleRef = useRef<LightningUiModule | null>(null)
  // wasm_ui_client.cc's main() reads window.LIGHTNING_WS_URL once at startup and runs forever
  // via emscripten_set_main_loop — there's no teardown path, so React StrictMode's dev-mode
  // double-invoke of effects would open a second websocket/canvas context onto the same
  // <canvas id="canvas">. Guard against that instead of trying to make the module unloadable.
  const startedRef = useRef(false)

  useEffect(() => {
    if (startedRef.current) return
    startedRef.current = true

    localStorage.setItem(WS_URL_STORAGE_KEY, wsUrl)
    ;(window as unknown as { LIGHTNING_WS_URL: string }).LIGHTNING_WS_URL = wsUrl

    nativeDynamicImport('/wasm/lightning_ui_wasm_client.mjs')
      .then((mod) => mod.default())
      .then((instance) => {
        moduleRef.current = instance
        instance.setFollow(follow)
        setStatus('running')
      })
      .catch((err: unknown) => {
        setStatus('error')
        setError(err instanceof Error ? err.message : String(err))
      })
    // eslint-disable-next-line react-hooks/exhaustive-deps -- only ever runs once, see startedRef
  }, [wsUrl])

  function handleFollowChange(e: React.ChangeEvent<HTMLInputElement>) {
    const next = e.target.checked
    setFollowState(next)
    moduleRef.current?.setFollow(next)
  }

  function handleResetView() {
    moduleRef.current?.resetView()
  }

  function handleReconnect(e: React.FormEvent) {
    e.preventDefault()
    const url = new URL(window.location.href)
    url.searchParams.set('ws', wsUrlInput)
    // main() only reads the server URL once at startup, so changing it means reloading
    // the whole module rather than trying to hot-swap the websocket underneath it.
    window.location.href = url.toString()
  }

  return (
    <div className="lightning-ui-viewer">
      <form className="lightning-ui-toolbar" onSubmit={handleReconnect}>
        <label htmlFor="ws-url">UiWireServer URL</label>
        <input
          id="ws-url"
          type="text"
          value={wsUrlInput}
          onChange={(e) => setWsUrlInput(e.target.value)}
          spellCheck={false}
        />
        <button type="submit">Reconnect</button>
        <label className="lightning-ui-follow">
          <input type="checkbox" checked={follow} disabled={status !== 'running'} onChange={handleFollowChange} />
          Follow
        </label>
        <button type="button" disabled={status !== 'running'} onClick={handleResetView}>
          Reset 3D View
        </button>
        <span className={`lightning-ui-status lightning-ui-status--${status}`}>
          {status === 'loading' && 'loading wasm module…'}
          {status === 'running' && `running — check console for websocket connect status (${wsUrl})`}
          {status === 'error' && `failed to load wasm module: ${error}`}
        </span>
      </form>
      <canvas id="canvas" width={1280} height={800} />
    </div>
  )
}
