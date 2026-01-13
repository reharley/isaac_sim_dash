FROM node:20-alpine AS builder

WORKDIR /app

# Install pnpm
RUN npm install -g pnpm@8

# Copy workspace files
COPY pnpm-workspace.yaml package.json pnpm-lock.yaml* ./
COPY apps apps/
COPY packages packages/

# Install dependencies
RUN pnpm install --frozen-lockfile

# Build
RUN pnpm build

# Production stage
FROM node:20-alpine

WORKDIR /app

RUN npm install -g pnpm@8

COPY --from=builder /app/pnpm-lock.yaml* ./
COPY --from=builder /app/apps/backend/package.json ./apps/backend/
COPY --from=builder /app/apps/backend/dist ./apps/backend/dist
COPY --from=builder /app/apps/frontend/dist ./apps/frontend/dist

# Install production dependencies only
RUN cd apps/backend && pnpm install --prod

EXPOSE 3001

CMD ["node", "apps/backend/dist/server.js"]
