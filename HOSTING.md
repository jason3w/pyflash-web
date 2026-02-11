# How to Host PyFlash Online (Free)

To make your flasher accessible to any user via a URL, the easiest way is to use **GitHub Pages**.

## Steps to Host on GitHub

1. **Create a new Repository** on GitHub (e.g., call it `pyflash`).
2. **Upload `web_flash.html`**:
   - Rename `web_flash.html` to `index.html` (this makes it the home page).
   - Upload it to your new repository.
3. **Enable GitHub Pages**:
   - Go to **Settings** > **Pages** in your repository.
   - Under "Build and deployment", set "Source" to **Deploy from a branch**.
   - Select your branch (usually `main`) and the `/ (root)` folder.
   - Click **Save**.
4. **Your URL is Ready!**
   - After a minute, GitHub will give you a link like `https://yourusername.github.io/pyflash/`.
   - Anyone with this link can now flash their devices directly from their browser!

---

## Why use this?
- **HTTPS by Default**: Required for WebUSB to work.
- **Zero Maintenance**: No servers to manage.
- **Global Access**: Fast and reliable for any user worldwide.
